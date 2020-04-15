/**********************************************************************
* COPYRIGHT 2015 MULTI-TECH SYSTEMS, INC.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*   1. Redistributions of source code must retain the above copyright notice,
*      this list of conditions and the following disclaimer.
*   2. Redistributions in binary form must reproduce the above copyright notice,
*      this list of conditions and the following disclaimer in the documentation
*      and/or other materials provided with the distribution.
*   3. Neither the name of MULTI-TECH SYSTEMS, INC. nor the names of its contributors
*      may be used to endorse or promote products derived from this software
*      without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
******************************************************************************
*/

#include "mbed.h"
#include "tinysh.h"
#include "config.h"
#include "sx1280-hal.h"
#include "Crypto.h"


#ifndef __MTS_LORA_COMMANDS__
#define __MTS_LORA_COMMANDS__

void tinyshell_thread();
void reset_func(int argc, char **argv);

void deveui_func(int argc, char **argv);
void appeui_func(int argc, char **argv);
void appkey_func(int argc, char **argv);

void channels_func(int argc, char **argv);
void channel_index_func(int argc, char **argv);
void datarate_func(int argc, char **argv);
void power_func(int argc, char **argv);

void join_func(int argc, char **argv);
void send_func(int argc, char **argv);
void sendi_func(int argc, char **argv);
void recv_func(int argc, char **argv);

void devaddr_func(int argc, char **argv);
void nwkskey_func(int argc, char **argv);
void appskey_func(int argc, char **argv);
void ucnt_func(int argc, char **argv);
void dcnt_func(int argc, char **argv);

void app_port_func(int argc, char **argv);

void InitApplication();
void SetupRadio();

#endif

