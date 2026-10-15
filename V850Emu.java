// V850Emu.java — emulate FDIM startup+init, then INJECT CAN frames into the AFCAN RX
// pipeline and diff RAM to map which variable each signal byte lands in.
// Firmware linked for flash base 0x4100 (load image at 0x4100). CALLT/CTRET hand-executed.
// @category FDIM
import ghidra.app.script.GhidraScript;
import ghidra.app.emulator.EmulatorHelper;
import ghidra.program.model.address.Address;
import java.math.BigInteger;

public class V850Emu extends GhidraScript {
    EmulatorHelper emu;
    long callts=0, ctrets=0;
    final long SENT = 0x7FFFFFFEL;
    boolean afcanStub = true;

    @Override
    public void run() throws Exception {
        long initSteps = 160000;
        if (getScriptArgs().length>0){ try{ initSteps=Long.parseLong(getScriptArgs()[0]); }catch(Exception e){} }
        emu = new EmulatorHelper(currentProgram);
        zero(0x03FF0000L,0x10000); zero(0x03FE0000L,0x10000); zero(0xFFFF0000L,0x10000);
        emu.writeMemory(toAddr(0xFFFFF6C2L), new byte[]{1});
        emu.writeRegister(emu.getPCRegister(), BigInteger.valueOf(0x4100L));
        wr("sp",0x03FFEFACL); wr("gp",0x03FF7C64L); wr("tp",0x0000B4A8L);

        println("[emu] init run, "+initSteps+" steps");
        long st=0;
        while (st<initSteps){ int r=advance(); if(r==2){ println("[emu] halt during init @0x"+hex(pc())); break; } if(r==0) st++; }
        println("[emu] init done steps="+st+" callt="+callts+" ctret="+ctrets+" pc=0x"+hex(pc()));

        // ---- AFCAN buffer MID map (which buffer carries which CAN id) ----
        println("\n[inj] AFCAN buffer MIDs after init:");
        int buf353=-1, buf128=-1, buf307=-1, buf313=-1, freeRx=-1;
        for (int n=0;n<32;n++){
            long base=0x3FEC100L+n*0x20;
            int midh = erd8(base+0x0C) | (erd8(base+0x0D)<<8);
            int id = (midh>>2) & 0x7FF;
            int cfg = erd8(base+9);
            if (midh!=0 || cfg!=0) println(String.format("  buf[%2d] MIDH=0x%04x id=0x%03x cfg=0x%02x",n,midh,id,cfg));
            if (id==0x353) buf353=n;
            if (id==0x128) buf128=n;
            if (id==0x307) buf307=n;
            if (id==0x313) buf313=n;
            if (cfg!=0 && id==0 && freeRx<0) freeRx=n;
        }
        println("[inj] buf353="+buf353+" buf128="+buf128+" buf307="+buf307+" buf313="+buf313+" freeRx="+freeRx);

        // Program buffer MIDs from the ROM table (flash 0x6a80->bin 0x2980) so routing is sane.
        for (int n=3;n<32;n++){ int midh=img8(0x6a80+(n-3)*2)|(img8(0x6a80+(n-3)*2+1)<<8);
            try{ emu.writeMemory(toAddr(0x3FEC10CL+n*0x20L), new byte[]{(byte)(midh&0xff),(byte)((midh>>8)&0xff)});}catch(Exception e){} }

        // ---- inject 0x128 (illum=byte1) at TWO levels; diff which SFR/var tracks it ----
        inject("0x128 illum=0x20", 29, 0x128, new int[]{0x00,0x20,0x00,0xFF,0x00,0x00,0x00,0x00});
        inject("0x128 illum=0xE0", 29, 0x128, new int[]{0x00,0xE0,0x00,0xFF,0x00,0x00,0x00,0x00});
        // ---- inject 0x353 (ambient=byte4) ----
        inject("0x353 ambient=0x5A", 12, 0x353, new int[]{0x11,0x22,0xA2,0xA3,0x5A,0xA5,0xA6,0x77});

        emu.dispose();
    }

    void inject(String label, int bufIdx, int canId, int[] data) {
        println("\n[inj] ===== "+label+" buf["+bufIdx+"] markers["+markers(data)+"] =====");
        long base=0x3FEC100L+bufIdx*0x20;
        byte[] ramB=snap(0x03FF0000L,0xC000), sfrB=snap(0xFFFFF000L,0x1000), ppaB=snap(0x03FEC000L,0x1000);
        try {
            for (int i=0;i<8;i++) emu.writeMemory(toAddr(base+i), new byte[]{(byte)data[i]});
            int midh=(canId<<2)&0xFFFF;
            emu.writeMemory(toAddr(base+0x0C), new byte[]{(byte)(midh&0xff),(byte)((midh>>8)&0xff)});
            emu.writeMemory(toAddr(base+0x08), new byte[]{8,0});
            emu.writeMemory(toAddr(base+0x0E), new byte[]{0x01,0});
        } catch(Exception e){ println("  setup err "+e); }
        boolean old=afcanStub; afcanStub=false;
        callFn(0x114C2L+0x4100L, bufIdx);
        long st=0; while(st<400000){ int r=advance(); if(r==2) break; if(r==0) st++; }   // let reactive update run
        afcanStub=old;
        diff("RAM",0x03FF0000L,ramB,snap(0x03FF0000L,0xC000),data);
        diff("SFR",0xFFFFF000L,sfrB,snap(0xFFFFF000L,0x1000),data);
        diff("PPA",0x03FEC000L,ppaB,snap(0x03FEC000L,0x1000),data);
    }
    void diff(String tag,long base,byte[]b,byte[]a,int[]data){
        int c=0; int b1=data[1]&0xff;
        for(int i=0;i<a.length && c<50;i++){
            if(a[i]!=b[i]){
                int nv=a[i]&0xff, ov=b[i]&0xff; String t="";
                for(int k=0;k<8;k++) if(data[k]!=0 && nv==(data[k]&0xff)) t+=" =frameB"+k;
                if(nv==((0xff-b1)&0xff)) t+=" =~(0xff-illum)";
                if(nv==((0x1f-(b1&0x1f))&0xff)) t+=" =~(0x1f-illum5)";
                println(String.format("    %s 0x%08x: %02x->%02x%s",tag,base+i,ov,nv,t)); c++;
            }
        }
        println("  "+tag+" changed="+countDiff(b,a));
    }

    String markers(int[] d){ StringBuilder s=new StringBuilder(); for(int i=0;i<8;i++) s.append("b"+i+"=0x"+hex(d[i]&0xff)+" "); return s.toString(); }
    int countDiff(byte[]a,byte[]b){ int c=0; for(int i=0;i<a.length;i++) if(a[i]!=b[i])c++; return c; }
    byte[] snap(long base,int len){ try{ return emu.readMemory(toAddr(base),len);}catch(Exception e){ return new byte[len]; } }

    void callFn(long entry, long arg){
        wr("r6", arg); wr("r31", SENT);
        emu.writeRegister(emu.getPCRegister(), BigInteger.valueOf(entry));
        long cap=4_000_000, st=0;
        while (st<cap){
            long p=pc();
            if (p==SENT || p==(SENT&0xFFFFFFFEL)){ println("  [call] returned after "+st+" steps"); return; }
            int r=advance();
            if (r==2){ println("  [call] halt @0x"+hex(pc())+" after "+st); return; }
            if (r==0) st++;
        }
        println("  [call] hit step cap, pc=0x"+hex(pc()));
    }

    // one engine step: handle CALLT/CTRET/AFCAN, else emu.step. returns 0=stepped,1=handled,2=halt
    int advance(){
        long p=pc();
        if (afcanStub && p>=0x14C00L && p<=0x15700L){
            try{ for(long n=0;n<32;n++) emu.writeMemory(toAddr(0x03FEC10EL+n*0x20L),new byte[]{0,0});
                 emu.writeMemory(toAddr(0x03FEC000L),new byte[]{0,0}); }catch(Exception e){}
        }
        int b0=img8(p), b1=img8(p+1); int hw=b0|(b1<<8);
        if ((hw&0xFFC0)==0x0200){ // CALLT
            int imm=hw&0x3F; long ctbp=reg("CTBP"); long ent=img16(ctbp+imm*2L);
            long tgt=(ctbp+ent)&0xFFFFFFFEL; wr("CTPC",(p+2)&0xFFFFFFFFL);
            emu.writeRegister(emu.getPCRegister(), BigInteger.valueOf(tgt)); callts++; return 1;
        }
        if (b0==0xE0 && b1==0x07 && img8(p+2)==0x44 && img8(p+3)==0x01){ // CTRET
            long ctpc=reg("CTPC"); emu.writeRegister(emu.getPCRegister(), BigInteger.valueOf(ctpc&0xFFFFFFFFL)); ctrets++; return 1;
        }
        if (p==SENT || p==(SENT&0xFFFFFFFEL)) return 2;
        if (p<0x4100L || (p>=0x40000L && p<0x03FF0000L)){ println("[emu] PC escaped 0x"+hex(p)); return 2; }
        try{ if(!emu.step(monitor)){ return 2; } }catch(Throwable t){ println("[emu] ex "+t.getMessage()+" @0x"+hex(p)); return 2; }
        return 0;
    }

    void wr(String n,long v){ try{ emu.writeRegister(n,BigInteger.valueOf(v&0xFFFFFFFFL)); }catch(Exception e){} }
    long reg(String n){ try{ return emu.readRegister(n).longValue()&0xFFFFFFFFL; }catch(Exception e){ return 0; } }
    void zero(long b,int l){ emu.writeMemory(toAddr(b),new byte[l]); }
    long pc(){ return emu.readRegister(emu.getPCRegister()).longValue()&0xFFFFFFFFL; }
    String hex(long v){ return Long.toHexString(v); }
    int img8(long a){ try{ return getByte(toAddr(a))&0xff;}catch(Exception e){ try{return emu.readMemory(toAddr(a),1)[0]&0xff;}catch(Exception e2){return 0;} } }
    long img16(long a){ return img8(a)|(img8(a+1)<<8); }
    int erd8(long a){ try{ return emu.readMemory(toAddr(a),1)[0]&0xff;}catch(Exception e){return 0;} }
}
