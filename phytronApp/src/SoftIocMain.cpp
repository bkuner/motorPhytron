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

#include <stddef.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <errno.h>

#include "epicsThread.h"
#include "iocsh.h"

int main(int argc,char *argv[])
{
/* Check for PIDFILE in environment and create PID file if set */
    FILE* pidfile;
    char* pidfilename = getenv("PIDFILE");

    if (pidfilename) {
        pidfile = fopen(pidfilename, "w");
        if (pidfile) {
            fprintf(pidfile, "%u\n", getpid());
            fclose(pidfile);
        } else {
            perror("Can't open PID file:");
        }
    }
    
    if(argc>=2) {
        iocsh(argv[1]);
        epicsThreadSleep(.2);
    }
    iocsh(NULL);
    return(0);
}
