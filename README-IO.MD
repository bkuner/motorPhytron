# PhytronIoCtrl Port

- Support for phyMotion digital and analog IO modules
- Command interface to the phyMotion controler by stringout -in records and IOC shell

## Setup and shell commands

Each IO-Card needs its own port. The number count is not the slot number in the device, but 
the number as used to identify the card in a phyMotion command. So the count of the first 
analog and digital card begin by 1.

Configuration is neccessary for Analog cards to set the mode - unipolar, bipolar..

``
    drvAsynIPPortConfigure("BOX1","193.149.12.226:22222",0,0,1)

    # Each card needs its own port. 
    #Arg0: IP-Port name
    #Arg1: PhytronIo port name
    #Arg2: Card number as used by the command 
    #Arg3: Timeout [ms]
    #Arg4: Configuration String
    phytronCreateIoCtrl( "BOX1","DIG1", 1, 200,"")
    phytronCreateIoCtrl( "BOX1","DIG2", 2, 200,"")
    # Set AIO modules to bipolar
    phytronCreateIoCtrl( "BOX1","AIO1", 1, 200,"DA1.1T1;DA1.2T1;DA1.3T1;DA1.4T1;AD1.1T1;AD1.2T1;AD1.3T1;AD1.4T1")
    phytronCreateIoCtrl( "BOX1","AIO2", 2, 200,"DA2.1T1;DA2.2T1;DA2.3T1;DA2.4T1;AD2.1T1;AD2.2T1;AD2.3T1;AD2.4T1")
``
Shell commands:

- ``phycmd <cmd>``: one or list of commands, Return `VALUE|ACK|NACK|ERR` for a single command, something
  strange for a list of commands.
- ``phytronReport``: show the card type of each slot in the device, NACK for empty slots.

## Support for these interfaces.

- n = card number as defined in the port
- m = address in the record link

* readInt32 reasons: 

  * DIN: Read digital port (Command 'EGnR')
  * AIN: Read analog port 1 (Command: 'ADn.m') .. 
  * DOUT: Readback digital output port (Command: 'AGnR') 
  * AOUT: Readback analog port 1  (Command: 'DAn.m') ..

* writeInt32 reasons: 

  * DOUT: Write digital port (Command: 'AGn.Sval') 
  * AOUT: Write analog port 1  (Command: 'DAn.m=val') ..

* readOctet, writeOctet:

  * CMD: stringout record will send an arbitrary command and store the response to be read by stringin record.

This string interface is intended to be used for sequencers and scripts to get
a communication channel to the phyMotion device. The response of a command will
contain the ``DATA`` or the strings ``ACK``, ``NACK`` if there is no data. ``ERR`` will
be set for a corruped message.

## Example:

``
    record(longin,"MO1:inDig") {
        field(DESC,"read digital port")
        field(DTYP,"asynInt32")
        field(INP,"@asyn(BOXIO,0)DIN")
        field(FLNK,"MO1:inAi1")
    }
    record(longout,"MO1:setDig") {
        field(DESC,"set digital port")
        field(DTYP,"asynInt32")
        field(OUT,"@asyn(BOXIO,0)DOUT")
    }
    record(longin,"MO1:rdbkDig") {
        field(DESC,"rdbk set value")
        field(DTYP,"asynInt32")
        field(INP,"@asyn(BOXIO,0)DOUT")
    }
    record(ao,"PHYIO:setVoltAIO2:1") {
        field(DESC,"set AIO2, bipolar output")
        field(DTYP,"asynInt32")
        field(OUT,"@asyn(AIO2,1)AOUT")
        field(LINR,"SLOPE")
        field(ESLO,"0.00030518")
        field(EOFF,"0")
        field(EGU ,"V")
        field(LOPR,"-10")
        field(HOPR,"10")
        field(PREC,"4")
        field(FLNK,"PHYIO:rdVoltAIO2:1")
    }
    record(ai,"PHYIO:rdVoltAIO2:1") {
        field(DESC,"rd AIO2 bipolar input")
        field(DTYP,"asynInt32")
        field(INP,"@asyn(AIO2,1)AIN")
        field(SCAN,"")
        field(LINR,"SLOPE")
        field(ESLO,"0.0012208")
        field(EOFF,"0")
        field(EGU ,"V")
        field(LOPR,"-10")
        field(HOPR,"10")
        field(PREC,"4")
        field(FLNK,"PHYIO:rdbkVoltAIO2:1")
    }


    # command interface
    record(stringout,"MO1:setCmd") {
        field(DESC,"MO1 Command")
        field(DTYP,"asynOctetWrite")
        field(OUT, "@asyn(BOXIO,0)CMD")
        field(FLNK,"MO1:rdCmd")
    }
    record(stringin,"MO1:rdCmd") {
        field(DESC,"cmd response")
        field(DTYP,"asynOctetRead")
        field(INP,"@asyn(BOXIO,0)CMD")
        field(FLNK,"MO1:rdIntCmd")
    }
    record(longin,"MO1:rdIntCmd") {
        field(DESC,"cmd response as int")
        field(INP,"MO1:rdCmd")
    }
``
