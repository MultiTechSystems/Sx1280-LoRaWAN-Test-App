/**********************************************************************
* COPYRIGHT 2019 MULTI-TECH SYSTEMS, INC.
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

#include "config.h"

// DEFAULT SETTINGS
uint8_t devEUI[8] = { 0x00 };
uint8_t appEUI[8] = { 0x00 };
uint8_t appKey[16] = { 0x00 };

ConfigManager::~ConfigManager() {

}


ConfigManager::ConfigManager()
{
    Wakeup();
    // Mount();
}

void ConfigManager::Sleep() {

}

void ConfigManager::Wakeup() {

}


void ConfigManager::Load(DeviceConfig_t& dc) {
    DefaultProtected(dc);    
    DefaultSettings(dc);
    DefaultSession(dc);
    Default(dc);
}

void ConfigManager::DefaultSettings(DeviceConfig_t& dc) {
    dc.app_settings.DutyCycleEnabled = false;
    dc.app_settings.TxInterval = 10000;
}

void ConfigManager::DefaultSession(DeviceConfig_t& dc) {

}

void ConfigManager::DefaultProtected(DeviceConfig_t& dc) {
    memcpy(dc.provisioning.DeviceEUI, devEUI, 8);
}

void ConfigManager::Default(DeviceConfig_t& dc) {
    memcpy(dc.settings.AppEUI, appEUI, 8);
    memcpy(dc.settings.AppKey, appKey, 16);
}
