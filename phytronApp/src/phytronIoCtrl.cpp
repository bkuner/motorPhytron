/*************************************************************************\
* Copyright (c) 2016 Helmholtz-Zentrum Berlin
*     fuer Materialien und Energie GmbH (HZB), Berlin, Germany.
*
*   This program is free software: you can redistribute it and/or modify
*   it under the terms of the GNU Lesser General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   This program is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with this program.  If not, see <http://www.gnu.org/licenses/>.
\*************************************************************************/
#include <stdlib.h>
#include <string.h>

#include <iostream>
#include <string>
#include <vector>

#include <epicsThread.h>
#include <iocsh.h>

#include <asynPortDriver.h>
#include <asynOctetSyncIO.h>
#include <epicsExport.h>
#define epicsExportSharedSymbols
#include <shareLib.h>
#include <cantProceed.h>
#include "phytronIoCtrl.h"

static const char *driverName = "asynPhytronIoCtrl";

/* set by the first module to be used in phytronReport */
static std::vector<phytronIoCtrl*> controllers;


#define STATE2STRMAX 6
const char *state2str[STATE2STRMAX] ={"Success","Timeout","Overflow","Error","Disconnected","Disabled"};

phytronIoCtrl* findController(const char *portName)
{
    phytronIoCtrl* ctr = NULL;
    uint32_t i;
    for(i = 0; i < controllers.size(); i++){
        if(!strcmp(controllers[i]->getControllerName(), portName)) {
            ctr = controllers[i];
            break;
      }
    }
   return ctr;
}

std::string trim(const std::string &s)
{
    std::string::const_iterator it = s.begin();
    while (it != s.end() && isspace(*it))
        it++;

    std::string::const_reverse_iterator rit = s.rbegin();
    while (rit.base() != it && isspace(*rit))
        rit++;

    return std::string(it, rit.base());
}

/** Creates a new phytronIoCtrl object.
  * Phytron IO commands address the first module,channel or bit by 1 not by 0!
  * - phytronIoCtrl cardNr means the number of the card as found on the bus, first analog: cardNr=1,
  *   first digital cardNr=1. The total amount of digital or analog cards may be found by the commands 'IMAIO','IMDIO'
  * - asynPortDriver maxAddress=9 to address the channels 1-n for analog cards or for digital cards addr=0 all
  *   bits of the port and addr. 1..n to access single bits.
  */
phytronIoCtrl::phytronIoCtrl(const char *portName, const char *asynPortName, int cardNr,int timeout)
  : asynPortDriver(portName, 9,
      asynOctetMask | asynInt32Mask | asynFloat64Mask | asynGenericPointerMask | asynDrvUserMask,
      asynOctetMask | asynInt32Mask | asynFloat64Mask | asynGenericPointerMask,
      ASYN_CANBLOCK | ASYN_MULTIDEVICE, NUM_PHYIO_PARAMS, 0, 0)
{
    static const char *functionName = "phytronIoCtrl";
    asynStatus status;
    timeout_ = timeout/1000.0;

    if(findController(portName) != NULL) {
        printf("%s: port '%s' allready defined, constructor failed\n", driverName, portName);
        return;
    }

    this->controllerName_ = (char *) mallocMustSucceed(sizeof(char)*(strlen(portName)+1),
        "phytronIoCtrl::phytronIoCtrl: Controller name memory allocation failed.\n");
    strcpy(this->controllerName_, portName);
    this->cardNr = cardNr;

    /* Create the base set of card parameters */
    createParam(dInString, asynParamInt32, &dIn_);
    createParam(ainString, asynParamInt32, &ain_);

    createParam(dOutString, asynParamInt32,&dOut_);
    createParam(aoutString, asynParamInt32,&aout_);
    createParam(cmdString, asynParamOctet, &cmd_);

    /* Connect to phytron controller */
    status = pasynOctetSyncIO->connect(asynPortName, 0, &pController_, NULL);
    if (status) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                  "%s: cannot connect to phytron controller\n",
                  functionName);
    }
    else
        controllers.push_back(this);

    asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: constructor complete\n", driverName, functionName);
}

static char outBuf[MAX_CONTROLLER_STRING_SIZE];
char* toHex(const char *in)
{
    int i;
    int inLen = (int) strlen(in);
    char   *set = outBuf;

    if( inLen*4 < MAX_CONTROLLER_STRING_SIZE-1) {
        sprintf(set,"x%02X",in[0]);
        set += 3;
        for(i=1;i<inLen;i++){
            sprintf(set,",x%02X",in[i]);
            set += 4 ;
        }

    }
    else
        *outBuf=0x0;
    return outBuf;
}

phytronIoCtrl::~phytronIoCtrl() {}

/** Writes a command string to the controller.
  * \param[in] output The string to be written.
  * \param[in] timeout Timeout before returning an error.*/
asynStatus phytronIoCtrl::writeController(const char *output, double timeout)
{
  size_t nwrite;
  asynStatus status;
  char cmdBuf[MAX_CONTROLLER_STRING_SIZE+6];
  char functionName[] = "phytronIoCtrl::writeController";

  sprintf(cmdBuf,"\x02\x30%s:XX\x03",output);
  status = pasynOctetSyncIO->write(pController_, cmdBuf,strlen(cmdBuf), timeout, &nwrite);
  asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW,"%s: cmd:'%s',write:%lu,[%s]:'%s'\n",
            functionName,output,nwrite,toHex(cmdBuf),cmdBuf);
  return status;
}

/* Command syntax: <STX><ADDR>comand:[CS|XX]<ETX> or <STX><ACK><ETX>
 * Response syntax:<STX><ACK>data:CS<ETX> or <STX><ACK><ETX>
 *
 * STX=0x2, ACK=0x6/0x15 acknowledge/not acknowledge, ADDR=0, ':'=seperator,
 * CS=Checksum or 'XX' to ignore checksum, <ETX>=0x3
*/
asynStatus phytronIoCtrl::writeReadController(asynUser *pasynUser, const char *value, size_t maxChars,char *data, int *acknowledge, size_t *response_len)
{
    char functionName[] = "phytronIoCtrl::writeReadController";
    asynStatus status = asynSuccess;

    char outBuf[MAX_CONTROLLER_STRING_SIZE+6];
    char inBuf[MAX_CONTROLLER_STRING_SIZE];
    size_t nwrite;
    int eomReason;

    if(strlen(value) >= MAX_CONTROLLER_STRING_SIZE) {
        status =asynError;
        if( status != lastStatus ) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s: send value size > MAX_CONTROLLER_STRING_SIZE (%s) \n",functionName,value);
            lastStatus = status;
            return status;
        }
    }
    sprintf(outBuf,"\x02\x30%s:XX\x03",value);
    status = pasynOctetSyncIO->writeRead(pController_, outBuf,strlen(outBuf), inBuf, MAX_CONTROLLER_STRING_SIZE+6, this->timeout_,&nwrite, response_len, &eomReason);
    if(status == asynSuccess) {
        char *sep, *parse;
        parse = inBuf;
        if( (*parse== 0x02) && (parse[(*response_len)-1] == 0x3)) {
            *acknowledge = (int) *(parse+1);
            if(*acknowledge == 0x6) {
                parse += 2;
                if(*parse != 0x3) {  // command just acknowledged, no data?
                    sep = strchr(parse,':');
                    //printf("%s: sep:%s parse:%s checksum:%d\n",functionName,sep,parse,atoi(sep+1));
                    if(sep != NULL) {
                        /*  checkSum = atoi(sep+1);*/
                        *sep = 0;
                        strcpy(data,parse);
                    }
                }
            }
        }
        else
            status = asynError;
    }
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"%s cmd '%s' write: [%s] response %lu,'%s'[%s]\n",
              functionName,value,toHex(outBuf),*response_len,this->cmdBuf,toHex(this->cmdBuf));

    if( (status == asynError) && (status != lastStatus) ) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s: Communication failed \n",functionName);
        lastStatus = status;
        return status;
    }
    lastStatus = asynSuccess;
    return status;

}

asynStatus phytronIoCtrl::writeOctet(asynUser *pasynUser, const char *value, size_t maxChars,size_t *nActual)
{
    char functionName[] = "phytronIoCtrl::writeOctet";
    asynStatus status = asynSuccess;
    size_t response_len;
    int acknowledge=0;
    char buf[MAX_CONTROLLER_STRING_SIZE];

    status = writeReadController(pasynUser,value,maxChars,buf,&acknowledge,&response_len);
    if(status == asynSuccess){
        *nActual = strlen(value);   /* satisfy writeOcted caller when be shure that write is done successfully */
        if(acknowledge == 0x6) {
            if(strlen(buf)>0)
                strcpy(this->cmdBuf,buf);
            else
                strcpy(this->cmdBuf,"ACK");
        }
        else if(acknowledge == 0x15)
            strcpy(this->cmdBuf,"NACK");
        else {
            strcpy(this->cmdBuf,"ERR");
            status = asynError;
        }
    }
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"%s write:'%s'\t response: %s\n",functionName,value,this->cmdBuf);

    if( (status == asynError) && (status != lastStatus) ) {
        asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,"%s: Communication failed \n",functionName);
        lastStatus = status;
    }
    else
        lastStatus = asynSuccess;
    return status;
}

asynStatus phytronIoCtrl::readOctet(asynUser *pasynUser, char *value, size_t maxChars,size_t *nActual, int *eomReason)
{
    asynStatus status = asynSuccess;
    char functionName[] = "phytronIoCtrl:readOctet";
    *nActual = strlen(this->cmdBuf);

    if(pasynUser->reason == cmd_) {
        if(*nActual > maxChars) {
            status = asynError;
            if(status == asynError){
                if (status != lastStatus) {
                    asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                              "%s: Failed with status %d for reason %d\n",functionName,status, pasynUser->reason);
                    lastStatus = status;
                }
            }
        }
        else {
            strcpy(value,this->cmdBuf);
            *eomReason = ASYN_EOM_EOS;
            asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"%s read: '%s'\n",functionName,value);
        }

        lastStatus = asynSuccess;
    }
    return status;
}
/** asynUsers use this to read integer parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[out] value      Parameter value
 */
asynStatus phytronIoCtrl::readInt32(asynUser *pasynUser, epicsInt32 *value)
{
    int chanNr,acknowledge;
    int reason = pasynUser->reason;
    size_t response_len;
    char outBuf[MAX_CONTROLLER_STRING_SIZE];
    char inBuf[MAX_CONTROLLER_STRING_SIZE];
    static const char *functionName = "readInt32";
    asynStatus status = getAddress(pasynUser, &chanNr);

    if(status == asynError) {
        if (status != lastStatus) {
            asynPrint(pasynUser, ASYN_TRACE_ERROR,
                      "%s:%s: %s invalid address=%d, must be in range 0 to %d\n",
                      driverName, functionName, portName, chanNr, this->maxAddr-1);
            lastStatus = status;
        }
        return status;
    }

    if(reason == dIn_)                      // read digital input
        if(chanNr==0)
            sprintf(outBuf, "EG%dR",this->cardNr);
        else
            sprintf(outBuf, "EZ%d.%d",this->cardNr,chanNr);
    else if(reason == dOut_)                // readback digital output
        if(chanNr==0)
            sprintf(outBuf, "AG%dR",this->cardNr);
        else
            sprintf(outBuf, "AZ%d.%d",this->cardNr,chanNr);
    else if(reason == ain_)
        sprintf(outBuf, "AD%d.%d",this->cardNr,chanNr);
    else if(reason == aout_)
        sprintf(outBuf, "DA%d.%d",this->cardNr,chanNr);
    else if(reason == cmd_) {
    }

    status = writeReadController(pasynUser, outBuf, MAX_CONTROLLER_STRING_SIZE,inBuf, &acknowledge, &response_len);
    if(acknowledge != 0x06)
        status = asynError;

    if(status == asynError){
        if (status != lastStatus) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s: Failed with status %d (%s) for reason %d card:%d.%d cmd:%s\n",
                      functionName, status, state2str[status],pasynUser->reason,this->cardNr,chanNr,outBuf);
            lastStatus = status;
        }
        return status;
    }
    *value = atoi(inBuf);
    asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"%s card%d.%d cmd:'%s' read: %d\n",functionName,this->cardNr,chanNr,outBuf,*value);
    lastStatus = asynSuccess;
    return status;
}

/** asynUsers use this to write integer parameters
 * \param[in] pasynUser   asynUser structure containing the reason
 * \param[in] value       Parameter value to be written
 */
asynStatus phytronIoCtrl::writeInt32(asynUser *pasynUser, epicsInt32 value)
{
  int chanNr;
  char outBuf[MAX_CONTROLLER_STRING_SIZE];
  int reason = pasynUser->reason;
  static const char *functionName = "writeInt32";
  asynStatus status = getAddress(pasynUser, &chanNr);

  if(status == asynError) {
      if (status != lastStatus) {
          asynPrint(pasynUser, ASYN_TRACE_ERROR,
                    "%s:%s: %s invalid address=%d, must be in range 0 to %d\n",
                    driverName, functionName, portName, chanNr, this->maxAddr-1);
          lastStatus = status;
      }
      return status;
  }

  if(reason == dOut_)
      if(chanNr==0)
          sprintf(outBuf, "AG%dS%d",this->cardNr,value);
      else
          sprintf(outBuf, "A%d.%d%c",this->cardNr,chanNr,value?'S':'R');
  else if(reason == aout_)
      sprintf(outBuf, "DA%d.%d=%d",this->cardNr,chanNr,value);
  asynPrint(this->pasynUserSelf, ASYN_TRACEIO_DRIVER,"%s: card:%d.%d reason:%d cmd: '%s'\n",functionName,this->cardNr,chanNr,pasynUser->reason,outBuf);
  status = writeController(outBuf, timeout_) ;
  if(status == asynError){
    if (status != lastStatus) {
      asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR,
              "%s: Failed with status %d for reason %d\n",functionName, status, pasynUser->reason);
      lastStatus = status;
    }
    return status;
  }
  lastStatus = asynSuccess;
  return status;
}
void phytronIoCtrl::report(FILE *fp, int level)
{
}

asynStatus phytronIoCtrl::setParam(const char *paramStr, int dbg)
{
    asynStatus status;
    char functionName[] = "setParam";
    int pos=0, fnd=0;
    std::string parse(paramStr);
    std::string delim(";");
    std::string param;
    std::string resp;
    char response[MAX_CONTROLLER_STRING_SIZE];

    if(strlen(paramStr)==0)
        return asynSuccess;

    if(dbg>0)
        printf("Command\t\tResponse\n");

    while(fnd != -1) {
        fnd = parse.find(delim,pos);
        param = parse.substr(pos,fnd-pos);
        param = trim(param);
        //std::cout << len << ", "<<pos << ", "<<fnd << ", '"<< param <<"'"<< std::endl;
        pos = fnd+1;

        status = cmd(param.c_str(), response, MAX_CONTROLLER_STRING_SIZE);
        if(dbg>0)
            printf("'%s'\t'%s'\t%s\n",param.c_str(),response,((status>0)&&(status<STATE2STRMAX))?state2str[status]:"");
        asynPrint(this->pasynUserSelf, ASYN_TRACE_FLOW, "%s:%s: set: %s, %s\n", driverName, functionName,param.c_str(),response);
        if(status != asynSuccess) {
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: set param failed: %s\n", driverName, functionName,
                      (status<STATE2STRMAX)?state2str[status]:"Illegal status");
        }
        resp = response;
        if(resp == "NACK" || resp == "ERR")
            asynPrint(this->pasynUserSelf, ASYN_TRACE_ERROR, "%s:%s: set param failed: '%s' %s\n",
                      driverName, functionName,param.c_str(),resp.c_str());

    }
    return status;
}

asynStatus phytronIoCtrl::cmd(const char *cmd, char*response, size_t MaxResponseLen)
{
    size_t nActual;
    asynStatus status = this->writeOctet(pController_, cmd, MaxResponseLen, &nActual);
    if(status == asynSuccess) {
        strncpy(response,this->cmdBuf,MaxResponseLen);
    }
    return status;
}
/** Creates a new phytronController object.
  * Configuration command, called directly or from iocsh
  * \param[in] portName          The name of the asyn port that will be created for this driver
  * \param[in] phytronPortName   The name of the drvAsynIPPPort that was created previously to connect to the phytron controller
  * \param[in] numController     number of cards that this controller supports
  */
extern "C" int phytronCreateIoCtrl(const char *phytronPortName, const char *asynPortName,
                                   int cardNr,int timeout,const char *configStr)
{
    phytronIoCtrl* controller = new phytronIoCtrl(asynPortName,phytronPortName,cardNr,timeout);
    if(controller != NULL){
        controller->setParam(configStr);
        return asynError;
    }
    return asynSuccess;
}

/** Parameters for iocsh phytron controller registration */
static const iocshArg phytronCreateIoCtrlArg0 = {"IP Port name", iocshArgString};
static const iocshArg phytronCreateIoCtrlArg1 = {"PhytronIo port name for this card", iocshArgString};
static const iocshArg phytronCreateIoCtrlArg2 = {"Number of this IO card [1..n]", iocshArgInt};
static const iocshArg phytronCreateIoCtrlArg3 = {"Timeout [ms]", iocshArgInt};
static const iocshArg phytronCreateIoCtrlArg4 = {"Module configuration", iocshArgString};
static const iocshArg * const phytronCreateIoCtrlArgs[] = {&phytronCreateIoCtrlArg0,
                                                           &phytronCreateIoCtrlArg1,
                                                           &phytronCreateIoCtrlArg2,
                                                           &phytronCreateIoCtrlArg3,
                                                           &phytronCreateIoCtrlArg4};

static const iocshFuncDef phytronCreateIoCtrlDef = {"phytronCreateIoCtrl", 5, phytronCreateIoCtrlArgs};

static void phytronCreateIoCtrlCallFunc(const iocshArgBuf *args)
{
  phytronCreateIoCtrl(args[0].sval, args[1].sval, args[2].ival, args[3].ival, args[4].sval);
}

/** Parameters for iocsh phytron controller registration */
static const iocshArg phytronReportArg0 = {"Port", iocshArgString};
static const iocshArg * const phytronReportArgs[] = {&phytronReportArg0};

static const iocshFuncDef phytronReportDef = {"phyreport", 1, phytronReportArgs};

static void phytronReport(const iocshArgBuf *args)
{
    int i;
    char cmd[MAX_CONTROLLER_STRING_SIZE];
    asynStatus status;
    phytronIoCtrl* controller = findController(args[0].sval);
    if(controller == NULL){
        printf("Cann't find controller '%s'\n",args[0].sval);
        return;
    }
    printf("phyMotion Device\nslot nr\t| card type:\n");
    for(i=1;i<=16;i++) {
        sprintf(cmd,"IM%d",i);
        status = controller->cmd(cmd,cmd,MAX_CONTROLLER_STRING_SIZE);
        if(status == asynSuccess)
            printf("  %d\t| %s\n", i, cmd);
        else
            printf("  %d\t| ERROR status: '%s' (%d)\n", i, (status<STATE2STRMAX)?state2str[status]:"Illegal status",status);
    }

}

/** Parameters for iocsh phytron controller registration */
static const iocshArg phycmdArg0 = {"Port", iocshArgString};
static const iocshArg phycmdArg1 = {"Command", iocshArgString};
static const iocshArg * const phycmdArgs[] = {&phycmdArg0,&phycmdArg1};

static const iocshFuncDef phycmdDef = {"phycmd", 2, phycmdArgs};

static void phycmd(const iocshArgBuf *args)
{
    char cmd[MAX_CONTROLLER_STRING_SIZE];
    asynStatus status;
    phytronIoCtrl* controller = findController(args[0].sval);
    if(controller == NULL){
        printf("Cann't find controller '%s'\n",args[0].sval);
        return;
    }
    if((strlen(args[0].sval) > 0) && (strlen(args[0].sval) < MAX_CONTROLLER_STRING_SIZE)) {
        status = controller->cmd(args[1].sval,cmd,MAX_CONTROLLER_STRING_SIZE);
        if(status == asynSuccess)
            printf("%s\n", cmd);
        else
            printf("ERROR status: '%s' (%d)\n", (status<STATE2STRMAX)?state2str[status]:"Illegal status",status);
    }
    else
        printf("ERROR illegal input\n");
}


static void phytronIoRegister(void)
{
    iocshRegister(&phytronCreateIoCtrlDef, phytronCreateIoCtrlCallFunc);
    iocshRegister(&phytronReportDef, phytronReport);
    iocshRegister(&phycmdDef, phycmd);
}

extern "C" {
epicsExportRegistrar(phytronIoRegister);
}
