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
#ifndef phytronIoCtrl_H
#define phytronIoCtrl_H

#include <string>
#include <epicsTypes.h>

#ifdef __cplusplus
#include <asynPortDriver.h>

#define MAX_CONTROLLER_STRING_SIZE 256
#define DEFAULT_CONTROLLER_TIMEOUT 2.0

#define NUM_PHYIO_PARAMS 5
#define dInString            "DIN"
#define ainString            "AIN"

#define dOutString           "DOUT"
#define aoutString           "AOUT"
#define cmdString            "CMD"

class phytronIoCtrl : public asynPortDriver {
public:
    phytronIoCtrl(const char *portName, const char *asynPortName, int numCards,int timeout);
    virtual ~phytronIoCtrl();
    virtual asynStatus readInt32(asynUser *pasynUser, epicsInt32 *value);
    virtual asynStatus readOctet(asynUser *pasynUser, char *value, size_t maxChars,size_t *nActual, int *eomReason);
    virtual asynStatus writeInt32(asynUser *pasynUser, epicsInt32 value);
    virtual asynStatus writeOctet(asynUser *pasynUser, const char *value, size_t maxChars,size_t *nActual);
    virtual void       report(FILE *fp, int level);
    char *getCmdBuf() {return cmdBuf;}
    char *getControllerName() {return controllerName_;}

    /* Functions for direct controller access, work with the first created port */
    asynStatus cmd(const char *cmd, char*response, size_t MaxResponseLen) ;
    asynStatus setParam(const char *paramStr, int dbg=0);
private:
    /* These are convenience functions for controllers that use asynOctet interfaces to the hardware */
    asynStatus writeController(const char *output, double timeout);
    asynStatus writeReadController(asynUser *pasynUser, const char *value, size_t maxChars,char *data, int *acknowledge, size_t *response_len);
    int cardNr;
    char * controllerName_;

    asynUser *pController_;

    int dIn_;
    int ain_;

    int dOut_;
    int aout_;

    int cmd_;
    char cmdBuf[MAX_CONTROLLER_STRING_SIZE]; // store last response of stringout writeRead

    double timeout_;
    asynStatus lastStatus;
};

phytronIoCtrl* findController(const char *portName);

#endif /* _cplusplus */
#endif /*  */
