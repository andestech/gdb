#as: -march=rv32ifv
#objdump: -dr

.*:[ 	]+file format .*


Disassembly of section .text:

0+000 <.text>:
[ 	]+0:[ 	]+00002573[ 	]+csrr[ 	]+a0,ustatus
[ 	]+4:[ 	]+00402573[ 	]+csrr[ 	]+a0,uie
[ 	]+8:[ 	]+00502573[ 	]+csrr[ 	]+a0,utvec
[ 	]+c:[ 	]+04002573[ 	]+csrr[ 	]+a0,uscratch
[ 	]+10:[ 	]+04102573[ 	]+csrr[ 	]+a0,uepc
[ 	]+14:[ 	]+04202573[ 	]+csrr[ 	]+a0,ucause
[ 	]+18:[ 	]+04302573[ 	]+csrr[ 	]+a0,utval
[ 	]+1c:[ 	]+04402573[ 	]+csrr[ 	]+a0,uip
[ 	]+20:[ 	]+00102573[ 	]+frflags[ 	]+a0
[ 	]+24:[ 	]+00202573[ 	]+frrm[ 	]+a0
[ 	]+28:[ 	]+00302573[ 	]+frsr[ 	]+a0
[ 	]+2c:[ 	]+c0002573[ 	]+rdcycle[ 	]+a0
[ 	]+30:[ 	]+c0102573[ 	]+rdtime[ 	]+a0
[ 	]+34:[ 	]+c0202573[ 	]+rdinstret[ 	]+a0
[ 	]+38:[ 	]+c0302573[ 	]+csrr[ 	]+a0,hpmcounter3
[ 	]+3c:[ 	]+c0402573[ 	]+csrr[ 	]+a0,hpmcounter4
[ 	]+40:[ 	]+c0502573[ 	]+csrr[ 	]+a0,hpmcounter5
[ 	]+44:[ 	]+c0602573[ 	]+csrr[ 	]+a0,hpmcounter6
[ 	]+48:[ 	]+c0702573[ 	]+csrr[ 	]+a0,hpmcounter7
[ 	]+4c:[ 	]+c0802573[ 	]+csrr[ 	]+a0,hpmcounter8
[ 	]+50:[ 	]+c0902573[ 	]+csrr[ 	]+a0,hpmcounter9
[ 	]+54:[ 	]+c0a02573[ 	]+csrr[ 	]+a0,hpmcounter10
[ 	]+58:[ 	]+c0b02573[ 	]+csrr[ 	]+a0,hpmcounter11
[ 	]+5c:[ 	]+c0c02573[ 	]+csrr[ 	]+a0,hpmcounter12
[ 	]+60:[ 	]+c0d02573[ 	]+csrr[ 	]+a0,hpmcounter13
[ 	]+64:[ 	]+c0e02573[ 	]+csrr[ 	]+a0,hpmcounter14
[ 	]+68:[ 	]+c0f02573[ 	]+csrr[ 	]+a0,hpmcounter15
[ 	]+6c:[ 	]+c1002573[ 	]+csrr[ 	]+a0,hpmcounter16
[ 	]+70:[ 	]+c1102573[ 	]+csrr[ 	]+a0,hpmcounter17
[ 	]+74:[ 	]+c1202573[ 	]+csrr[ 	]+a0,hpmcounter18
[ 	]+78:[ 	]+c1302573[ 	]+csrr[ 	]+a0,hpmcounter19
[ 	]+7c:[ 	]+c1402573[ 	]+csrr[ 	]+a0,hpmcounter20
[ 	]+80:[ 	]+c1502573[ 	]+csrr[ 	]+a0,hpmcounter21
[ 	]+84:[ 	]+c1602573[ 	]+csrr[ 	]+a0,hpmcounter22
[ 	]+88:[ 	]+c1702573[ 	]+csrr[ 	]+a0,hpmcounter23
[ 	]+8c:[ 	]+c1802573[ 	]+csrr[ 	]+a0,hpmcounter24
[ 	]+90:[ 	]+c1902573[ 	]+csrr[ 	]+a0,hpmcounter25
[ 	]+94:[ 	]+c1a02573[ 	]+csrr[ 	]+a0,hpmcounter26
[ 	]+98:[ 	]+c1b02573[ 	]+csrr[ 	]+a0,hpmcounter27
[ 	]+9c:[ 	]+c1c02573[ 	]+csrr[ 	]+a0,hpmcounter28
[ 	]+a0:[ 	]+c1d02573[ 	]+csrr[ 	]+a0,hpmcounter29
[ 	]+a4:[ 	]+c1e02573[ 	]+csrr[ 	]+a0,hpmcounter30
[ 	]+a8:[ 	]+c1f02573[ 	]+csrr[ 	]+a0,hpmcounter31
[ 	]+ac:[ 	]+c8002573[ 	]+rdcycleh[ 	]+a0
[ 	]+b0:[ 	]+c8102573[ 	]+rdtimeh[ 	]+a0
[ 	]+b4:[ 	]+c8202573[ 	]+rdinstreth[ 	]+a0
[ 	]+b8:[ 	]+c8302573[ 	]+csrr[ 	]+a0,hpmcounter3h
[ 	]+bc:[ 	]+c8402573[ 	]+csrr[ 	]+a0,hpmcounter4h
[ 	]+c0:[ 	]+c8502573[ 	]+csrr[ 	]+a0,hpmcounter5h
[ 	]+c4:[ 	]+c8602573[ 	]+csrr[ 	]+a0,hpmcounter6h
[ 	]+c8:[ 	]+c8702573[ 	]+csrr[ 	]+a0,hpmcounter7h
[ 	]+cc:[ 	]+c8802573[ 	]+csrr[ 	]+a0,hpmcounter8h
[ 	]+d0:[ 	]+c8902573[ 	]+csrr[ 	]+a0,hpmcounter9h
[ 	]+d4:[ 	]+c8a02573[ 	]+csrr[ 	]+a0,hpmcounter10h
[ 	]+d8:[ 	]+c8b02573[ 	]+csrr[ 	]+a0,hpmcounter11h
[ 	]+dc:[ 	]+c8c02573[ 	]+csrr[ 	]+a0,hpmcounter12h
[ 	]+e0:[ 	]+c8d02573[ 	]+csrr[ 	]+a0,hpmcounter13h
[ 	]+e4:[ 	]+c8e02573[ 	]+csrr[ 	]+a0,hpmcounter14h
[ 	]+e8:[ 	]+c8f02573[ 	]+csrr[ 	]+a0,hpmcounter15h
[ 	]+ec:[ 	]+c9002573[ 	]+csrr[ 	]+a0,hpmcounter16h
[ 	]+f0:[ 	]+c9102573[ 	]+csrr[ 	]+a0,hpmcounter17h
[ 	]+f4:[ 	]+c9202573[ 	]+csrr[ 	]+a0,hpmcounter18h
[ 	]+f8:[ 	]+c9302573[ 	]+csrr[ 	]+a0,hpmcounter19h
[ 	]+fc:[ 	]+c9402573[ 	]+csrr[ 	]+a0,hpmcounter20h
[ 	]+100:[ 	]+c9502573[ 	]+csrr[ 	]+a0,hpmcounter21h
[ 	]+104:[ 	]+c9602573[ 	]+csrr[ 	]+a0,hpmcounter22h
[ 	]+108:[ 	]+c9702573[ 	]+csrr[ 	]+a0,hpmcounter23h
[ 	]+10c:[ 	]+c9802573[ 	]+csrr[ 	]+a0,hpmcounter24h
[ 	]+110:[ 	]+c9902573[ 	]+csrr[ 	]+a0,hpmcounter25h
[ 	]+114:[ 	]+c9a02573[ 	]+csrr[ 	]+a0,hpmcounter26h
[ 	]+118:[ 	]+c9b02573[ 	]+csrr[ 	]+a0,hpmcounter27h
[ 	]+11c:[ 	]+c9c02573[ 	]+csrr[ 	]+a0,hpmcounter28h
[ 	]+120:[ 	]+c9d02573[ 	]+csrr[ 	]+a0,hpmcounter29h
[ 	]+124:[ 	]+c9e02573[ 	]+csrr[ 	]+a0,hpmcounter30h
[ 	]+128:[ 	]+c9f02573[ 	]+csrr[ 	]+a0,hpmcounter31h
[ 	]+12c:[ 	]+10002573[ 	]+csrr[ 	]+a0,sstatus
[ 	]+130:[ 	]+10202573[ 	]+csrr[ 	]+a0,sedeleg
[ 	]+134:[ 	]+10302573[ 	]+csrr[ 	]+a0,sideleg
[ 	]+138:[ 	]+10402573[ 	]+csrr[ 	]+a0,sie
[ 	]+13c:[ 	]+10502573[ 	]+csrr[ 	]+a0,stvec
[ 	]+140:[ 	]+14002573[ 	]+csrr[ 	]+a0,sscratch
[ 	]+144:[ 	]+14102573[ 	]+csrr[ 	]+a0,sepc
[ 	]+148:[ 	]+14202573[ 	]+csrr[ 	]+a0,scause
[ 	]+14c:[ 	]+14302573[ 	]+csrr[ 	]+a0,stval
[ 	]+150:[ 	]+14402573[ 	]+csrr[ 	]+a0,sip
[ 	]+154:[ 	]+18002573[ 	]+csrr[ 	]+a0,satp
[ 	]+158:[ 	]+20002573[ 	]+csrr[ 	]+a0,hstatus
[ 	]+15c:[ 	]+20202573[ 	]+csrr[ 	]+a0,hedeleg
[ 	]+160:[ 	]+20302573[ 	]+csrr[ 	]+a0,hideleg
[ 	]+164:[ 	]+20402573[ 	]+csrr[ 	]+a0,hie
[ 	]+168:[ 	]+20502573[ 	]+csrr[ 	]+a0,htvec
[ 	]+16c:[ 	]+24002573[ 	]+csrr[ 	]+a0,hscratch
[ 	]+170:[ 	]+24102573[ 	]+csrr[ 	]+a0,hepc
[ 	]+174:[ 	]+24202573[ 	]+csrr[ 	]+a0,hcause
[ 	]+178:[ 	]+24302573[ 	]+csrr[ 	]+a0,hbadaddr
[ 	]+17c:[ 	]+24402573[ 	]+csrr[ 	]+a0,hip
[ 	]+180:[ 	]+f1102573[ 	]+csrr[ 	]+a0,mvendorid
[ 	]+184:[ 	]+f1202573[ 	]+csrr[ 	]+a0,marchid
[ 	]+188:[ 	]+f1302573[ 	]+csrr[ 	]+a0,mimpid
[ 	]+18c:[ 	]+f1402573[ 	]+csrr[ 	]+a0,mhartid
[ 	]+190:[ 	]+30002573[ 	]+csrr[ 	]+a0,mstatus
[ 	]+194:[ 	]+30102573[ 	]+csrr[ 	]+a0,misa
[ 	]+198:[ 	]+30202573[ 	]+csrr[ 	]+a0,medeleg
[ 	]+19c:[ 	]+30302573[ 	]+csrr[ 	]+a0,mideleg
[ 	]+1a0:[ 	]+30402573[ 	]+csrr[ 	]+a0,mie
[ 	]+1a4:[ 	]+30502573[ 	]+csrr[ 	]+a0,mtvec
[ 	]+1a8:[ 	]+34002573[ 	]+csrr[ 	]+a0,mscratch
[ 	]+1ac:[ 	]+34102573[ 	]+csrr[ 	]+a0,mepc
[ 	]+1b0:[ 	]+34202573[ 	]+csrr[ 	]+a0,mcause
[ 	]+1b4:[ 	]+34302573[ 	]+csrr[ 	]+a0,mtval
[ 	]+1b8:[ 	]+34402573[ 	]+csrr[ 	]+a0,mip
[ 	]+1bc:[ 	]+38002573[ 	]+csrr[ 	]+a0,mbase
[ 	]+1c0:[ 	]+38102573[ 	]+csrr[ 	]+a0,mbound
[ 	]+1c4:[ 	]+38202573[ 	]+csrr[ 	]+a0,mibase
[ 	]+1c8:[ 	]+38302573[ 	]+csrr[ 	]+a0,mibound
[ 	]+1cc:[ 	]+38402573[ 	]+csrr[ 	]+a0,mdbase
[ 	]+1d0:[ 	]+38502573[ 	]+csrr[ 	]+a0,mdbound
[ 	]+1d4:[ 	]+b0002573[ 	]+csrr[ 	]+a0,mcycle
[ 	]+1d8:[ 	]+b0202573[ 	]+csrr[ 	]+a0,minstret
[ 	]+1dc:[ 	]+b0302573[ 	]+csrr[ 	]+a0,mhpmcounter3
[ 	]+1e0:[ 	]+b0402573[ 	]+csrr[ 	]+a0,mhpmcounter4
[ 	]+1e4:[ 	]+b0502573[ 	]+csrr[ 	]+a0,mhpmcounter5
[ 	]+1e8:[ 	]+b0602573[ 	]+csrr[ 	]+a0,mhpmcounter6
[ 	]+1ec:[ 	]+b0702573[ 	]+csrr[ 	]+a0,mhpmcounter7
[ 	]+1f0:[ 	]+b0802573[ 	]+csrr[ 	]+a0,mhpmcounter8
[ 	]+1f4:[ 	]+b0902573[ 	]+csrr[ 	]+a0,mhpmcounter9
[ 	]+1f8:[ 	]+b0a02573[ 	]+csrr[ 	]+a0,mhpmcounter10
[ 	]+1fc:[ 	]+b0b02573[ 	]+csrr[ 	]+a0,mhpmcounter11
[ 	]+200:[ 	]+b0c02573[ 	]+csrr[ 	]+a0,mhpmcounter12
[ 	]+204:[ 	]+b0d02573[ 	]+csrr[ 	]+a0,mhpmcounter13
[ 	]+208:[ 	]+b0e02573[ 	]+csrr[ 	]+a0,mhpmcounter14
[ 	]+20c:[ 	]+b0f02573[ 	]+csrr[ 	]+a0,mhpmcounter15
[ 	]+210:[ 	]+b1002573[ 	]+csrr[ 	]+a0,mhpmcounter16
[ 	]+214:[ 	]+b1102573[ 	]+csrr[ 	]+a0,mhpmcounter17
[ 	]+218:[ 	]+b1202573[ 	]+csrr[ 	]+a0,mhpmcounter18
[ 	]+21c:[ 	]+b1302573[ 	]+csrr[ 	]+a0,mhpmcounter19
[ 	]+220:[ 	]+b1402573[ 	]+csrr[ 	]+a0,mhpmcounter20
[ 	]+224:[ 	]+b1502573[ 	]+csrr[ 	]+a0,mhpmcounter21
[ 	]+228:[ 	]+b1602573[ 	]+csrr[ 	]+a0,mhpmcounter22
[ 	]+22c:[ 	]+b1702573[ 	]+csrr[ 	]+a0,mhpmcounter23
[ 	]+230:[ 	]+b1802573[ 	]+csrr[ 	]+a0,mhpmcounter24
[ 	]+234:[ 	]+b1902573[ 	]+csrr[ 	]+a0,mhpmcounter25
[ 	]+238:[ 	]+b1a02573[ 	]+csrr[ 	]+a0,mhpmcounter26
[ 	]+23c:[ 	]+b1b02573[ 	]+csrr[ 	]+a0,mhpmcounter27
[ 	]+240:[ 	]+b1c02573[ 	]+csrr[ 	]+a0,mhpmcounter28
[ 	]+244:[ 	]+b1d02573[ 	]+csrr[ 	]+a0,mhpmcounter29
[ 	]+248:[ 	]+b1e02573[ 	]+csrr[ 	]+a0,mhpmcounter30
[ 	]+24c:[ 	]+b1f02573[ 	]+csrr[ 	]+a0,mhpmcounter31
[ 	]+250:[ 	]+b8002573[ 	]+csrr[ 	]+a0,mcycleh
[ 	]+254:[ 	]+b8202573[ 	]+csrr[ 	]+a0,minstreth
[ 	]+258:[ 	]+b8302573[ 	]+csrr[ 	]+a0,mhpmcounter3h
[ 	]+25c:[ 	]+b8402573[ 	]+csrr[ 	]+a0,mhpmcounter4h
[ 	]+260:[ 	]+b8502573[ 	]+csrr[ 	]+a0,mhpmcounter5h
[ 	]+264:[ 	]+b8602573[ 	]+csrr[ 	]+a0,mhpmcounter6h
[ 	]+268:[ 	]+b8702573[ 	]+csrr[ 	]+a0,mhpmcounter7h
[ 	]+26c:[ 	]+b8802573[ 	]+csrr[ 	]+a0,mhpmcounter8h
[ 	]+270:[ 	]+b8902573[ 	]+csrr[ 	]+a0,mhpmcounter9h
[ 	]+274:[ 	]+b8a02573[ 	]+csrr[ 	]+a0,mhpmcounter10h
[ 	]+278:[ 	]+b8b02573[ 	]+csrr[ 	]+a0,mhpmcounter11h
[ 	]+27c:[ 	]+b8c02573[ 	]+csrr[ 	]+a0,mhpmcounter12h
[ 	]+280:[ 	]+b8d02573[ 	]+csrr[ 	]+a0,mhpmcounter13h
[ 	]+284:[ 	]+b8e02573[ 	]+csrr[ 	]+a0,mhpmcounter14h
[ 	]+288:[ 	]+b8f02573[ 	]+csrr[ 	]+a0,mhpmcounter15h
[ 	]+28c:[ 	]+b9002573[ 	]+csrr[ 	]+a0,mhpmcounter16h
[ 	]+290:[ 	]+b9102573[ 	]+csrr[ 	]+a0,mhpmcounter17h
[ 	]+294:[ 	]+b9202573[ 	]+csrr[ 	]+a0,mhpmcounter18h
[ 	]+298:[ 	]+b9302573[ 	]+csrr[ 	]+a0,mhpmcounter19h
[ 	]+29c:[ 	]+b9402573[ 	]+csrr[ 	]+a0,mhpmcounter20h
[ 	]+2a0:[ 	]+b9502573[ 	]+csrr[ 	]+a0,mhpmcounter21h
[ 	]+2a4:[ 	]+b9602573[ 	]+csrr[ 	]+a0,mhpmcounter22h
[ 	]+2a8:[ 	]+b9702573[ 	]+csrr[ 	]+a0,mhpmcounter23h
[ 	]+2ac:[ 	]+b9802573[ 	]+csrr[ 	]+a0,mhpmcounter24h
[ 	]+2b0:[ 	]+b9902573[ 	]+csrr[ 	]+a0,mhpmcounter25h
[ 	]+2b4:[ 	]+b9a02573[ 	]+csrr[ 	]+a0,mhpmcounter26h
[ 	]+2b8:[ 	]+b9b02573[ 	]+csrr[ 	]+a0,mhpmcounter27h
[ 	]+2bc:[ 	]+b9c02573[ 	]+csrr[ 	]+a0,mhpmcounter28h
[ 	]+2c0:[ 	]+b9d02573[ 	]+csrr[ 	]+a0,mhpmcounter29h
[ 	]+2c4:[ 	]+b9e02573[ 	]+csrr[ 	]+a0,mhpmcounter30h
[ 	]+2c8:[ 	]+b9f02573[ 	]+csrr[ 	]+a0,mhpmcounter31h
[ 	]+2cc:[ 	]+32002573[ 	]+csrr[ 	]+a0,mucounteren
[ 	]+2d0:[ 	]+32102573[ 	]+csrr[ 	]+a0,mscounteren
[ 	]+2d4:[ 	]+32202573[ 	]+csrr[ 	]+a0,mhcounteren
[ 	]+2d8:[ 	]+32302573[ 	]+csrr[ 	]+a0,mhpmevent3
[ 	]+2dc:[ 	]+32402573[ 	]+csrr[ 	]+a0,mhpmevent4
[ 	]+2e0:[ 	]+32502573[ 	]+csrr[ 	]+a0,mhpmevent5
[ 	]+2e4:[ 	]+32602573[ 	]+csrr[ 	]+a0,mhpmevent6
[ 	]+2e8:[ 	]+32702573[ 	]+csrr[ 	]+a0,mhpmevent7
[ 	]+2ec:[ 	]+32802573[ 	]+csrr[ 	]+a0,mhpmevent8
[ 	]+2f0:[ 	]+32902573[ 	]+csrr[ 	]+a0,mhpmevent9
[ 	]+2f4:[ 	]+32a02573[ 	]+csrr[ 	]+a0,mhpmevent10
[ 	]+2f8:[ 	]+32b02573[ 	]+csrr[ 	]+a0,mhpmevent11
[ 	]+2fc:[ 	]+32c02573[ 	]+csrr[ 	]+a0,mhpmevent12
[ 	]+300:[ 	]+32d02573[ 	]+csrr[ 	]+a0,mhpmevent13
[ 	]+304:[ 	]+32e02573[ 	]+csrr[ 	]+a0,mhpmevent14
[ 	]+308:[ 	]+32f02573[ 	]+csrr[ 	]+a0,mhpmevent15
[ 	]+30c:[ 	]+33002573[ 	]+csrr[ 	]+a0,mhpmevent16
[ 	]+310:[ 	]+33102573[ 	]+csrr[ 	]+a0,mhpmevent17
[ 	]+314:[ 	]+33202573[ 	]+csrr[ 	]+a0,mhpmevent18
[ 	]+318:[ 	]+33302573[ 	]+csrr[ 	]+a0,mhpmevent19
[ 	]+31c:[ 	]+33402573[ 	]+csrr[ 	]+a0,mhpmevent20
[ 	]+320:[ 	]+33502573[ 	]+csrr[ 	]+a0,mhpmevent21
[ 	]+324:[ 	]+33602573[ 	]+csrr[ 	]+a0,mhpmevent22
[ 	]+328:[ 	]+33702573[ 	]+csrr[ 	]+a0,mhpmevent23
[ 	]+32c:[ 	]+33802573[ 	]+csrr[ 	]+a0,mhpmevent24
[ 	]+330:[ 	]+33902573[ 	]+csrr[ 	]+a0,mhpmevent25
[ 	]+334:[ 	]+33a02573[ 	]+csrr[ 	]+a0,mhpmevent26
[ 	]+338:[ 	]+33b02573[ 	]+csrr[ 	]+a0,mhpmevent27
[ 	]+33c:[ 	]+33c02573[ 	]+csrr[ 	]+a0,mhpmevent28
[ 	]+340:[ 	]+33d02573[ 	]+csrr[ 	]+a0,mhpmevent29
[ 	]+344:[ 	]+33e02573[ 	]+csrr[ 	]+a0,mhpmevent30
[ 	]+348:[ 	]+33f02573[ 	]+csrr[ 	]+a0,mhpmevent31
[ 	]+34c:[ 	]+7a002573[ 	]+csrr[ 	]+a0,tselect
[ 	]+350:[ 	]+7a102573[ 	]+csrr[ 	]+a0,tdata1
[ 	]+354:[ 	]+7a202573[ 	]+csrr[ 	]+a0,tdata2
[ 	]+358:[ 	]+7a302573[ 	]+csrr[ 	]+a0,tdata3
[ 	]+35c:[ 	]+7b002573[ 	]+csrr[ 	]+a0,dcsr
[ 	]+360:[ 	]+7b102573[ 	]+csrr[ 	]+a0,dpc
[ 	]+364:[ 	]+7b202573[ 	]+csrr[ 	]+a0,dscratch0
[ 	]+368:[ 	]+04302573[ 	]+csrr[ 	]+a0,utval
[ 	]+36c:[ 	]+10602573[ 	]+csrr[ 	]+a0,scounteren
[ 	]+370:[ 	]+14302573[ 	]+csrr[ 	]+a0,stval
[ 	]+374:[ 	]+18002573[ 	]+csrr[ 	]+a0,satp
[ 	]+378:[ 	]+30602573[ 	]+csrr[ 	]+a0,mcounteren
[ 	]+37c:[ 	]+34302573[ 	]+csrr[ 	]+a0,mtval
[ 	]+380:[ 	]+3a002573[ 	]+csrr[ 	]+a0,pmpcfg0
[ 	]+384:[ 	]+3a102573[ 	]+csrr[ 	]+a0,pmpcfg1
[ 	]+388:[ 	]+3a202573[ 	]+csrr[ 	]+a0,pmpcfg2
[ 	]+38c:[ 	]+3a302573[ 	]+csrr[ 	]+a0,pmpcfg3
[ 	]+390:[ 	]+3b002573[ 	]+csrr[ 	]+a0,pmpaddr0
[ 	]+394:[ 	]+3b102573[ 	]+csrr[ 	]+a0,pmpaddr1
[ 	]+398:[ 	]+3b202573[ 	]+csrr[ 	]+a0,pmpaddr2
[ 	]+39c:[ 	]+3b302573[ 	]+csrr[ 	]+a0,pmpaddr3
[ 	]+3a0:[ 	]+3b402573[ 	]+csrr[ 	]+a0,pmpaddr4
[ 	]+3a4:[ 	]+3b502573[ 	]+csrr[ 	]+a0,pmpaddr5
[ 	]+3a8:[ 	]+3b602573[ 	]+csrr[ 	]+a0,pmpaddr6
[ 	]+3ac:[ 	]+3b702573[ 	]+csrr[ 	]+a0,pmpaddr7
[ 	]+3b0:[ 	]+3b802573[ 	]+csrr[ 	]+a0,pmpaddr8
[ 	]+3b4:[ 	]+3b902573[ 	]+csrr[ 	]+a0,pmpaddr9
[ 	]+3b8:[ 	]+3ba02573[ 	]+csrr[ 	]+a0,pmpaddr10
[ 	]+3bc:[ 	]+3bb02573[ 	]+csrr[ 	]+a0,pmpaddr11
[ 	]+3c0:[ 	]+3bc02573[ 	]+csrr[ 	]+a0,pmpaddr12
[ 	]+3c4:[ 	]+3bd02573[ 	]+csrr[ 	]+a0,pmpaddr13
[ 	]+3c8:[ 	]+3be02573[ 	]+csrr[ 	]+a0,pmpaddr14
[ 	]+3cc:[ 	]+3bf02573[ 	]+csrr[ 	]+a0,pmpaddr15
[ 	]+3d0+:[ 	]+00802573[ 	]+csrr[ 	]+a0,vstart
[ 	]+3d4+:[ 	]+00902573[ 	]+csrr[ 	]+a0,vxsat
[ 	]+3d8+:[ 	]+00a02573[ 	]+csrr[ 	]+a0,vxrm
[ 	]+3dc+:[ 	]+00f02573[ 	]+csrr[ 	]+a0,vcsr
[ 	]+3e0+:[ 	]+c2002573[ 	]+csrr[ 	]+a0,vl
[ 	]+3e4+:[ 	]+c2102573[ 	]+csrr[ 	]+a0,vtype
