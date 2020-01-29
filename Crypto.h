/* ______                              _
  / _____)             _              | |
 ( (____  _____ ____ _| |_ _____  ____| |__
  \____ \| ___ |    (_   _) ___ |/ ___)  _ \
  _____) ) ____| | | || |_| ____( (___| | | |
 (______/|_____)_|_|_| \__)_____)\____)_| |_|
 (C)2013 Semtech

 Description: LoRa MAC layer implementation

 License: Revised BSD License, see LICENSE.TXT file include in the project

 Maintainer: Miguel Luis and Gregory Cristian
 */
#ifndef __LORA_CRYPTO_H__
#define __LORA_CRYPTO_H__

#include "casado/aes.h"
#include "gladman/cmac.h"
#include <cstring>
#include <inttypes.h>

namespace lora {

    class Crypto
    {
        public:
            Crypto(void);

            /*!
             * Computes the LoRaMAC frame MIC field
             *
             * \param [IN]  buffer          Data buffer
             * \param [IN]  size            Data buffer size
             * \param [IN]  key             AES key to be used
             * \param [IN]  address         Frame address
             * \param [IN]  dir             Frame direction [0: uplink, 1: downlink, 2:peer]
             * \param [IN]  sequenceCounter Frame sequence counter
             * \param [OUT] mic             Computed MIC field
             */
            void ComputeMic(uint8_t *buffer, uint16_t size, uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint32_t *mic);

            /*!
             * Computes the LoRaMAC payload encryption
             *
             * \param [IN]  buffer          Data buffer
             * \param [IN]  size            Data buffer size
             * \param [IN]  key             AES key to be used
             * \param [IN]  address         Frame address
             * \param [IN]  dir             Frame direction [0: uplink, 1: downlink, 2:peer]
             * \param [IN]  sequenceCounter Frame sequence counter
             * \param [OUT] encBuffer       Encrypted buffer
             */
            void PayloadEncrypt(const uint8_t *buffer, uint16_t size, uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *encBuffer);

            /*!
             * Computes the LoRaMAC payload decryption
             *
             * \param [IN]  buffer          Data buffer
             * \param [IN]  size            Data buffer size
             * \param [IN]  key             AES key to be used
             * \param [IN]  address         Frame address
             * \param [IN]  dir             Frame direction [0: uplink, 1: downlink, 2:peer]
             * \param [IN]  sequenceCounter Frame sequence counter
             * \param [OUT] decBuffer       Decrypted buffer
             */
            void PayloadDecrypt(uint8_t *buffer, uint16_t size, uint8_t *key, uint32_t address, uint8_t dir, uint32_t sequenceCounter, uint8_t *decBuffer);

            /*!
             * Computes the LoRaMAC Join Request frame MIC field
             *
             * \param [IN]  buffer          Data buffer
             * \param [IN]  size            Data buffer size
             * \param [IN]  key             AES key to be used
             * \param [OUT] mic             Computed MIC field
             */
            void JoinComputeMic(const uint8_t *buffer, uint16_t size, const uint8_t *key, uint32_t *mic);

            /*!
             * Computes the LoRaMAC join frame decryption
             *
             * \param [IN]  buffer          Data buffer
             * \param [IN]  size            Data buffer size
             * \param [IN]  key             AES key to be used
             * \param [OUT] decBuffer       Decrypted buffer
             */
            void JoinDecrypt(uint8_t *buffer, uint16_t size, uint8_t *key, uint8_t *decBuffer);

            /*!
             * Computes the LoRaMAC join frame decryption
             *
             * \param [IN]  key             AES key to be used
             * \param [IN]  appNonce        Application nonce
             * \param [IN]  netId           Network ID
             * \param [IN]  devNonce        Device nonce
             * \param [OUT] nwkSKey         Network session key
             * \param [OUT] appSKey         Application session key
             */
            void DeriveSessionKeys(uint8_t *key, uint8_t *appNonce, uint8_t *netID, uint16_t devNonce, uint8_t *nwkSKey, uint8_t *appSKey);


            /*!
             * Helper to copy mic bytes onto buffer
             * \param [IN]  mic         integer mic value
             * \param [IN]  buff        location to copy to
             */
            void CopyMicToArray(uint32_t mic, uint8_t* buff);

            void DeriveMcKEKey(uint8_t *mcKEKey, uint8_t *appKey);

            void DecryptMcKey(uint8_t *mcKey, uint8_t *mcKeyEncrypt, uint8_t *mcKEKey);

            void DeriveMcSessionKeys(uint8_t *mcAppKey, uint8_t *mcNetKey, uint8_t *mcKey, uint8_t *mcAddr);

        private:
            /*!
             * MIC field computation initial data
             */
            uint8_t MicBlockB0[16];

            /*!
             * Contains the computed MIC field.
             *
             * \remark Only the 4 first bytes are used
             */
            uint8_t Mic[16];

            /*!
             * Encryption aBlock and sBlock
             */
            uint8_t aBlock[16];
            uint8_t sBlock[16];

            /*!
             * AES computation context variable
             */
            aes_context AesContext;

            /*!
             * CMAC computation context variable
             */
            AES_CMAC_CTX AesCmacCtx[1];
    };

}
#endif // __LORAMAC_CRYPTO_H__
