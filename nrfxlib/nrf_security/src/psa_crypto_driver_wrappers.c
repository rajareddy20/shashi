/*
 *  Functions to delegate cryptographic operations to an available
 *  and appropriate accelerator.
 *  Warning: This file will be auto-generated in the future.
 */
/*  Copyright The Mbed TLS Contributors
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#include "common.h"
#include "psa_crypto_aead.h"
#include "psa_crypto_cipher.h"
#include "psa_crypto_core.h"
#include "psa_crypto_driver_wrappers.h"
#include "psa_crypto_hash.h"
#include "psa_crypto_mac.h"
#include <string.h>

#include "mbedtls/platform.h"

#if defined(MBEDTLS_PSA_CRYPTO_C)

#if defined(MBEDTLS_PSA_CRYPTO_DRIVERS)

#if defined(PSA_CRYPTO_DRIVER_CC3XX)
#ifndef PSA_CRYPTO_DRIVER_PRESENT
#define PSA_CRYPTO_DRIVER_PRESENT
#endif
#ifndef PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT
#define PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT
#endif
#include "cc3xx.h"
#endif /* PSA_CRYPTO_DRIVER_CC3XX */

#if defined(PSA_CRYPTO_DRIVER_OBERON)
#ifndef PSA_CRYPTO_DRIVER_PRESENT
#define PSA_CRYPTO_DRIVER_PRESENT
#endif
#ifndef PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT
#define PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT
#endif
#include "oberon.h"
#endif /* PSA_CRYPTO_DRIVER_OBERON */


/* Repeat above block for each JSON-declared driver during autogeneration */
#endif /* MBEDTLS_PSA_CRYPTO_DRIVERS */

/* Auto-generated values depending on which drivers are registered.
 * ID 0 is reserved for unallocated operations.
 * ID 1 is reserved for the Mbed TLS software driver. */
#define PSA_CRYPTO_MBED_TLS_DRIVER_ID (1)

#if defined(PSA_CRYPTO_DRIVER_CC3XX)
#define PSA_CRYPTO_CC3XX_DRIVER_ID (4)
#endif /* PSA_CRYPTO_DRIVER_CC3XX */

#if defined(PSA_CRYPTO_DRIVER_OBERON)
#define PSA_CRYPTO_OBERON_DRIVER_ID (5)
#endif /* PSA_CRYPTO_DRIVER_OBERON */

/* Support the 'old' SE interface when asked to */
#if defined(MBEDTLS_PSA_CRYPTO_SE_C)
/* PSA_CRYPTO_DRIVER_PRESENT is defined when either a new-style or old-style
 * SE driver is present, to avoid unused argument errors at compile time. */
#ifndef PSA_CRYPTO_DRIVER_PRESENT
#define PSA_CRYPTO_DRIVER_PRESENT
#endif
#include "psa_crypto_se.h"
#endif

psa_status_t psa_driver_wrapper_init( void )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;

#if defined(MBEDTLS_PSA_CRYPTO_SE_C)
    status = psa_init_all_se_drivers( );
    if( status != PSA_SUCCESS )
        return( status );
#endif

    (void) status;
    return( PSA_SUCCESS );
}

void psa_driver_wrapper_free( void )
{
#if defined(MBEDTLS_PSA_CRYPTO_SE_C)
    /* Unregister all secure element drivers, so that we restart from
     * a pristine state. */
    psa_unregister_all_se_drivers( );
#endif /* MBEDTLS_PSA_CRYPTO_SE_C */
}

/* Start delegation functions */
psa_status_t psa_driver_wrapper_sign_message(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer,
    size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *input,
    size_t input_length,
    uint8_t *signature,
    size_t signature_size,
    size_t *signature_length )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_CC3XX)
            status = cc3xx_sign_message(
                        attributes,
                        key_buffer,
                        key_buffer_size,
                        alg,
                        input,
                        input_length,
                        signature,
                        signature_size,
                        signature_length );
            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_OBERON)
            status = oberon_sign_message(
                        attributes,
                        key_buffer,
                        key_buffer_size,
                        alg,
                        input,
                        input_length,
                        signature,
                        signature_size,
                        signature_length );
            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
            break;
        default:
            /* Key is declared with a lifetime not known to us */
            (void)status;
            break;
    }
#if defined(MBEDTLS_PSA_BUILTIN_HAS_ASYM_SIGN_SUPPORT)
    status = psa_sign_message_builtin( attributes,
                                      key_buffer,
                                      key_buffer_size,
                                      alg,
                                      input,
                                      input_length,
                                      signature,
                                      signature_size,
                                      signature_length );
    if( status != PSA_ERROR_NOT_SUPPORTED )
        return( status );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_ASYM_SIGN_SUPPORT */
    (void)attributes;
    (void)key_buffer;
    (void)key_buffer_size;
    (void)alg;
    (void)input;
    (void)input_length;
    (void)signature;
    (void)signature_size;
    (void)signature_length;
    return( PSA_ERROR_NOT_SUPPORTED );
}

psa_status_t psa_driver_wrapper_verify_message(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer,
    size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *input,
    size_t input_length,
    const uint8_t *signature,
    size_t signature_length )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_CC3XX)
            status = cc3xx_verify_message(
                        attributes,
                        key_buffer,
                        key_buffer_size,
                        alg,
                        input,
                        input_length,
                        signature,
                        signature_length );
            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_OBERON)
            status = oberon_verify_message(
                        attributes,
                        key_buffer,
                        key_buffer_size,
                        alg,
                        input,
                        input_length,
                        signature,
                        signature_length );
            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
            break;
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
        default:
            /* Key is declared with a lifetime not known to us */
            (void)status;
            break;
    }

#if defined(MBEDTLS_PSA_BUILTIN_HAS_ASYM_SIGN_SUPPORT)
    status = psa_verify_message_builtin( attributes,
                                         key_buffer,
                                         key_buffer_size,
                                         alg,
                                         input,
                                         input_length,
                                         signature,
                                         signature_length );
    if (status != PSA_ERROR_NOT_SUPPORTED)
        return status;
#endif /* MBEDTLS_PSA_BUILTIN_HAS_ASYM_SIGN_SUPPORT */
    (void)attributes;
    (void)key_buffer;
    (void)key_buffer_size;
    (void)alg;
    (void)input;
    (void)input_length;
    (void)signature;
    (void)signature_length;
    return( PSA_ERROR_NOT_SUPPORTED );
}

psa_status_t psa_driver_wrapper_sign_hash(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg, const uint8_t *hash, size_t hash_length,
    uint8_t *signature, size_t signature_size, size_t *signature_length )
{
    /* Try dynamically-registered SE interface first */
#if defined(MBEDTLS_PSA_CRYPTO_SE_C)
    const psa_drv_se_t *drv;
    psa_drv_se_context_t *drv_context;

    if( psa_get_se_driver( attributes->core.lifetime, &drv, &drv_context ) )
    {
        if( drv->asymmetric == NULL ||
            drv->asymmetric->p_sign == NULL )
        {
            /* Key is defined in SE, but we have no way to exercise it */
            return( PSA_ERROR_NOT_SUPPORTED );
        }
        return( drv->asymmetric->p_sign(
                    drv_context, *( (psa_key_slot_number_t *)key_buffer ),
                    alg, hash, hash_length,
                    signature, signature_size, signature_length ) );
    }
#endif /* PSA_CRYPTO_SE_C */

    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_CC3XX)
            status = cc3xx_sign_hash( attributes,
                                      key_buffer,
                                      key_buffer_size,
                                      alg,
                                      hash,
                                      hash_length,
                                      signature,
                                      signature_size,
                                      signature_length );
            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_OBERON)
            status = oberon_sign_hash( attributes,
                                       key_buffer,
                                       key_buffer_size,
                                       alg,
                                       hash,
                                       hash_length,
                                       signature,
                                       signature_size,
                                       signature_length );
            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
            /* Fell through, meaning no accelerator supports this operation */
#if defined(MBEDTLS_PSA_BUILTIN_HAS_ASYM_SIGN_SUPPORT)
            status = psa_sign_hash_builtin( attributes,
                                            key_buffer,
                                            key_buffer_size,
                                            alg,
                                            hash,
                                            hash_length,
                                            signature,
                                            signature_size,
                                            signature_length );
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_ASYM_SIGN_SUPPORT */
            /* Fell through, meaning nothing supports this operation */
            (void)attributes;
            (void)key_buffer;
            (void)key_buffer_size;
            (void)alg;
            (void)hash;
            (void)hash_length;
            (void)signature;
            (void)signature_size;
            (void)signature_length;
            return( PSA_ERROR_NOT_SUPPORTED );
        /* Add cases for opaque driver here */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_TEST)
        case PSA_CRYPTO_TEST_DRIVER_LOCATION:
            return( mbedtls_test_opaque_signature_sign_hash( attributes,
                                                             key_buffer,
                                                             key_buffer_size,
                                                             alg,
                                                             hash,
                                                             hash_length,
                                                             signature,
                                                             signature_size,
                                                             signature_length ) );
#endif /* PSA_CRYPTO_DRIVER_TEST */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
        default:
            /* Key is declared with a lifetime not known to us */
            (void)status;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_verify_hash(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg, const uint8_t *hash, size_t hash_length,
    const uint8_t *signature, size_t signature_length )
{
    /* Try dynamically-registered SE interface first */
#if defined(MBEDTLS_PSA_CRYPTO_SE_C)
    const psa_drv_se_t *drv;
    psa_drv_se_context_t *drv_context;

    if( psa_get_se_driver( attributes->core.lifetime, &drv, &drv_context ) )
    {
        if( drv->asymmetric == NULL ||
            drv->asymmetric->p_verify == NULL )
        {
            /* Key is defined in SE, but we have no way to exercise it */
            return( PSA_ERROR_NOT_SUPPORTED );
        }
        return( drv->asymmetric->p_verify(
                    drv_context, *( (psa_key_slot_number_t *)key_buffer ),
                    alg, hash, hash_length,
                    signature, signature_length ) );
    }
#endif /* PSA_CRYPTO_SE_C */

    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_CC3XX)
            /* Do not call the cc3xx_verify_hash for RSA keys since it still in early development */
            status = cc3xx_verify_hash( attributes,
                                        key_buffer,
                                        key_buffer_size,
                                        alg,
                                        hash,
                                        hash_length,
                                        signature,
                                        signature_length );
            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_OBERON)
            status = oberon_verify_hash(attributes,
                                        key_buffer,
                                        key_buffer_size,
                                        alg,
                                        hash,
                                        hash_length,
                                        signature,
                                        signature_length);

            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ASYM_SIGN_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
#if defined(MBEDTLS_PSA_BUILTIN_HAS_ASYM_SIGN_SUPPORT)
            return( psa_verify_hash_builtin( attributes,
                                             key_buffer,
                                             key_buffer_size,
                                             alg,
                                             hash,
                                             hash_length,
                                             signature,
                                             signature_length ) );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_ASYM_SIGN_SUPPORT */
            (void) attributes;
            (void) key_buffer;
            (void) key_buffer_size;
            (void) alg;
            (void) hash;
            (void) hash_length;
            (void) signature;
            (void) signature_length;
        default:
            /* Key is declared with a lifetime not known to us */
            (void)status;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

/** Calculate the key buffer size required to store the key material of a key
 *  associated with an opaque driver from input key data.
 *
 * \param[in] attributes        The key attributes
 * \param[in] data              The input key data.
 * \param[in] data_length       The input data length.
 * \param[out] key_buffer_size  Minimum buffer size to contain the key material.
 *
 * \retval #PSA_SUCCESS
 * \retval #PSA_ERROR_INVALID_ARGUMENT
 * \retval #PSA_ERROR_NOT_SUPPORTED
 */
psa_status_t psa_driver_wrapper_get_key_buffer_size_from_key_data(
    const psa_key_attributes_t *attributes,
    const uint8_t *data,
    size_t data_length,
    size_t *key_buffer_size )
{
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );
    psa_key_type_t key_type = attributes->core.type;

    *key_buffer_size = 0;
    switch( location )
    {
        default:
            (void)key_type;
            (void)data;
            (void)data_length;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

/** Get the key buffer size required to store the key material of a key
 *  associated with an opaque driver.
 *
 * \param[in] attributes  The key attributes.
 * \param[out] key_buffer_size  Minimum buffer size to contain the key material
 *
 * \retval #PSA_SUCCESS
 *         The minimum size for a buffer to contain the key material has been
 *         returned successfully.
 * \retval #PSA_ERROR_NOT_SUPPORTED
 *         The type and/or the size in bits of the key or the combination of
 *         the two is not supported.
 * \retval #PSA_ERROR_INVALID_ARGUMENT
 *         The key is declared with a lifetime not known to us.
 */
psa_status_t psa_driver_wrapper_get_key_buffer_size(
    const psa_key_attributes_t *attributes,
    size_t *key_buffer_size )
{
    psa_key_location_t location = PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );
    psa_key_type_t key_type = attributes->core.type;
    size_t key_bits = attributes->core.bits;

    *key_buffer_size = 0;
    switch( location )
    {
        default:
            (void)key_type;
            (void)key_bits;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_generate_key(
    const psa_key_attributes_t *attributes,
    uint8_t *key_buffer, size_t key_buffer_size, size_t *key_buffer_length )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION(attributes->core.lifetime);

    /* Try dynamically-registered SE interface first */
#if defined(MBEDTLS_PSA_CRYPTO_SE_C)
    const psa_drv_se_t *drv;
    psa_drv_se_context_t *drv_context;

    if( psa_get_se_driver( attributes->core.lifetime, &drv, &drv_context ) )
    {
        size_t pubkey_length = 0; /* We don't support this feature yet */
        if( drv->key_management == NULL ||
            drv->key_management->p_generate == NULL )
        {
            /* Key is defined as being in SE, but we have no way to generate it */
            return( PSA_ERROR_NOT_SUPPORTED );
        }
        return( drv->key_management->p_generate(
            drv_context,
            *( (psa_key_slot_number_t *)key_buffer ),
            attributes, NULL, 0, &pubkey_length ) );
    }
#endif /* MBEDTLS_PSA_CRYPTO_SE_C */

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
            /* Transparent drivers are limited to generating asymmetric keys */
            if( PSA_KEY_TYPE_IS_ASYMMETRIC( attributes->core.type ) )
            {
            /* Cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_DRIVER_HAS_ACCEL_KEY_TYPES_CC3XX)
                status = cc3xx_generate_key(
                    attributes, key_buffer, key_buffer_size,
                    key_buffer_length );
                /* Declared with fallback == true */
                if( status != PSA_ERROR_NOT_SUPPORTED )
                    break;
#endif /* PSA_CRYPTO_DRIVER_HAS_ACCEL_KEY_TYPES_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_ACCEL_KEY_TYPES_OBERON)
                status = oberon_generate_key(
                    attributes, key_buffer, key_buffer_size,
                    key_buffer_length );
                /* Declared with fallback == true */
                if( status != PSA_ERROR_NOT_SUPPORTED )
                    break;
#endif /* PSA_CRYPTO_DRIVER_HAS_ACCEL_KEY_TYPES_OBERON */
            }
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */

            /* Software fallback */
            status = psa_generate_key_internal(
                attributes, key_buffer, key_buffer_size, key_buffer_length );
            break;

        default:
            /* Key is declared with a lifetime not known to us */
            status = PSA_ERROR_INVALID_ARGUMENT;
            break;
    }

    return( status );
}

psa_status_t psa_driver_wrapper_import_key(
    const psa_key_attributes_t *attributes,
    const uint8_t *data,
    size_t data_length,
    uint8_t *key_buffer,
    size_t key_buffer_size,
    size_t *key_buffer_length,
    size_t *bits )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location = PSA_KEY_LIFETIME_GET_LOCATION(
                                      psa_get_key_lifetime( attributes ) );

    /* Try dynamically-registered SE interface first */
#if defined(MBEDTLS_PSA_CRYPTO_SE_C)
    const psa_drv_se_t *drv;
    psa_drv_se_context_t *drv_context;

    if( psa_get_se_driver( attributes->core.lifetime, &drv, &drv_context ) )
    {
        if( drv->key_management == NULL ||
            drv->key_management->p_import == NULL )
            return( PSA_ERROR_NOT_SUPPORTED );

        /* The driver should set the number of key bits, however in
         * case it doesn't, we initialize bits to an invalid value. */
        *bits = PSA_MAX_KEY_BITS + 1;
        status = drv->key_management->p_import(
            drv_context,
            *( (psa_key_slot_number_t *)key_buffer ),
            attributes, data, data_length, bits );

        if( status != PSA_SUCCESS )
            return( status );

        if( (*bits) > PSA_MAX_KEY_BITS )
            return( PSA_ERROR_NOT_SUPPORTED );

        return( PSA_SUCCESS );
    }
#endif /* PSA_CRYPTO_SE_C */

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_ACCEL_KEY_TYPES_CC3XX)
           status = cc3xx_import_key(
                         attributes,
                         data, data_length,
                         key_buffer, key_buffer_size,
                         key_buffer_length, bits );
           /* Declared with fallback == true */
           if( status != PSA_ERROR_NOT_SUPPORTED )
               return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ACCEL_KEY_TYPES_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_ACCEL_KEY_TYPES_OBERON)
           status = oberon_import_key(
                         attributes,
                         data, data_length,
                         key_buffer, key_buffer_size,
                         key_buffer_length, bits );
           /* Declared with fallback == true */
           if( status != PSA_ERROR_NOT_SUPPORTED )
               return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ACCEL_KEY_TYPES_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
            /* Fell through, meaning no accelerator supports this operation */
#if defined(MBEDTLS_PSA_BUILTIN_HAS_KEY_TYPE)
            return( psa_import_key_into_slot( attributes,
                                              data, data_length,
                                              key_buffer, key_buffer_size,
                                              key_buffer_length, bits ) );
#else
            return ( PSA_ERROR_NOT_SUPPORTED );
#endif /* !MBEDTLS_PSA_BUILTIN_HAS_KEY_TYPE */
        default:
            (void)status;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }

}

psa_status_t psa_driver_wrapper_export_key(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    uint8_t *data, size_t data_size, size_t *data_length )

{
    psa_status_t status = PSA_ERROR_INVALID_ARGUMENT;
    psa_key_location_t location = PSA_KEY_LIFETIME_GET_LOCATION(
                                      psa_get_key_lifetime( attributes ) );

    /* Try dynamically-registered SE interface first */
#if defined(MBEDTLS_PSA_CRYPTO_SE_C)
    const psa_drv_se_t *drv;
    psa_drv_se_context_t *drv_context;

    if( psa_get_se_driver( attributes->core.lifetime, &drv, &drv_context ) )
    {
        if( ( drv->key_management == NULL   ) ||
            ( drv->key_management->p_export == NULL ) )
        {
            return( PSA_ERROR_NOT_SUPPORTED );
        }

        return( drv->key_management->p_export(
                     drv_context,
                     *( (psa_key_slot_number_t *)key_buffer ),
                     data, data_size, data_length ) );
    }
#endif /* PSA_CRYPTO_SE_C */

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            return( psa_export_key_internal( attributes,
                                             key_buffer,
                                             key_buffer_size,
                                             data,
                                             data_size,
                                             data_length ) );
        default:
            /* Key is declared with a lifetime not known to us */
            return( status );
    }
}

psa_status_t psa_driver_wrapper_export_public_key(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    uint8_t *data, size_t data_size, size_t *data_length )

{
    psa_status_t status = PSA_ERROR_INVALID_ARGUMENT;
    psa_key_location_t location = PSA_KEY_LIFETIME_GET_LOCATION(
                                      psa_get_key_lifetime( attributes ) );

    /* Try dynamically-registered SE interface first */
#if defined(MBEDTLS_PSA_CRYPTO_SE_C)
    const psa_drv_se_t *drv;
    psa_drv_se_context_t *drv_context;

    if( psa_get_se_driver( attributes->core.lifetime, &drv, &drv_context ) )
    {
        if( ( drv->key_management == NULL ) ||
            ( drv->key_management->p_export_public == NULL ) )
        {
            return( PSA_ERROR_NOT_SUPPORTED );
        }

        return( drv->key_management->p_export_public(
                    drv_context,
                    *( (psa_key_slot_number_t *)key_buffer ),
                    data, data_size, data_length ) );
    }
#endif /* MBEDTLS_PSA_CRYPTO_SE_C */

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_ACCEL_KEY_TYPES_CC3XX)
            status = cc3xx_export_public_key(
                         attributes,
                         key_buffer,
                         key_buffer_size,
                         data,
                         data_size,
                         data_length );
            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ACCEL_KEY_TYPES_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_ACCEL_KEY_TYPES_OBERON)
             status = oberon_export_public_key(
                          attributes,
                          key_buffer,
                          key_buffer_size,
                          data,
                          data_size,
                          data_length );
             /* Declared with fallback == true */
             if( status != PSA_ERROR_NOT_SUPPORTED )
                 return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ACCEL_KEY_TYPES_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
#if defined(MBEDTLS_PSA_BUILTIN_HAS_KEY_TYPE)
            /* Fell through, meaning no accelerator supports this operation */
            return( psa_export_public_key_internal( attributes,
                                                    key_buffer,
                                                    key_buffer_size,
                                                    data,
                                                    data_size,
                                                    data_length ) );
#else
            return ( PSA_ERROR_NOT_SUPPORTED );
#endif /* !MBEDTLS_PSA_BUILTIN_HAS_KEY_TYPE */
        default:
            /* Key is declared with a lifetime not known to us */
            return( status );
    }
}

psa_status_t psa_driver_wrapper_get_builtin_key(
    psa_drv_slot_number_t slot_number,
    psa_key_attributes_t *attributes,
    uint8_t *key_buffer, size_t key_buffer_size, size_t *key_buffer_length )
{
    psa_key_location_t location = PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );
    switch( location )
    {
        default:
            (void) slot_number;
            (void) key_buffer;
            (void) key_buffer_size;
            (void) key_buffer_length;
            return( PSA_ERROR_DOES_NOT_EXIST );
    }
}

psa_status_t psa_driver_wrapper_copy_key(
    psa_key_attributes_t *attributes,
    const uint8_t *source_key, size_t source_key_length,
    uint8_t *target_key_buffer, size_t target_key_buffer_size,
    size_t *target_key_buffer_length )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

#if defined(MBEDTLS_PSA_CRYPTO_SE_C)
    const psa_drv_se_t *drv;
    psa_drv_se_context_t *drv_context;

    if( psa_get_se_driver( attributes->core.lifetime, &drv, &drv_context ) )
    {
        /* Copying to a secure element is not implemented yet. */
        return( PSA_ERROR_NOT_SUPPORTED );
    }
#endif /* MBEDTLS_PSA_CRYPTO_SE_C */

    switch( location )
    {
        default:
            (void)source_key;
            (void)source_key_length;
            (void)target_key_buffer;
            (void)target_key_buffer_size;
            (void)target_key_buffer_length;
            status = PSA_ERROR_INVALID_ARGUMENT;
    }
    return( status );
}

/*
 * Cipher functions
 */
psa_status_t psa_driver_wrapper_cipher_encrypt(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer,
    size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *iv,
    size_t iv_length,
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_size,
    size_t *output_length )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX)
            status = cc3xx_cipher_encrypt( attributes,
                                           key_buffer,
                                           key_buffer_size,
                                           alg,
                                           iv,
                                           iv_length,
                                           input,
                                           input_length,
                                           output,
                                           output_size,
                                           output_length );

            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON)
            status = oberon_cipher_encrypt( attributes,
                                            key_buffer,
                                            key_buffer_size,
                                            alg,
                                            iv,
                                            iv_length,
                                            input,
                                            input_length,
                                            output,
                                            output_size,
                                            output_length );

            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */

#if defined(MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT)
            return( mbedtls_psa_cipher_encrypt( attributes,
                                                key_buffer,
                                                key_buffer_size,
                                                alg,
                                                iv,
                                                iv_length,
                                                input,
                                                input_length,
                                                output,
                                                output_size,
                                                output_length ) );
#else
            (void) attributes;
            (void) key_buffer;
            (void) key_buffer_size;
            (void) alg;
            (void) iv;
            (void) iv_length;
            (void) input;
            (void) input_length;
            (void) output;
            (void) output_size;
            (void) output_length;
            return( PSA_ERROR_NOT_SUPPORTED );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT */
        default:
            /* Key is declared with a lifetime not known to us */
            (void)status;
            (void)key_buffer;
            (void)key_buffer_size;
            (void)alg;
            (void)iv;
            (void)iv_length;
            (void)input;
            (void)input_length;
            (void)output;
            (void)output_size;
            (void)output_length;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_cipher_decrypt(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer,
    size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_size,
    size_t *output_length )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX)
            status = cc3xx_cipher_decrypt( attributes,
                                           key_buffer,
                                           key_buffer_size,
                                           alg,
                                           input,
                                           input_length,
                                           output,
                                           output_size,
                                           output_length );
            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON)
           status = oberon_cipher_decrypt( attributes,
                                           key_buffer,
                                           key_buffer_size,
                                           alg,
                                           input,
                                           input_length,
                                           output,
                                           output_size,
                                           output_length );
           /* Declared with fallback == true */
           if( status != PSA_ERROR_NOT_SUPPORTED )
               return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */

#if defined(MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT)
            return( mbedtls_psa_cipher_decrypt( attributes,
                                                key_buffer,
                                                key_buffer_size,
                                                alg,
                                                input,
                                                input_length,
                                                output,
                                                output_size,
                                                output_length ) );
#else
            return( PSA_ERROR_NOT_SUPPORTED );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT */
        default:
            /* Key is declared with a lifetime not known to us */
            (void)status;
            (void)key_buffer;
            (void)key_buffer_size;
            (void)alg;
            (void)input;
            (void)input_length;
            (void)output;
            (void)output_size;
            (void)output_length;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_cipher_encrypt_setup(
    psa_cipher_operation_t *operation,
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX)
            status = cc3xx_cipher_encrypt_setup(
                &operation->ctx.cc3xx_driver_ctx,
                attributes,
                key_buffer,
                key_buffer_size,
                alg );
            /* Declared with fallback == true */
            if( status == PSA_SUCCESS )
                operation->id = PSA_CRYPTO_CC3XX_DRIVER_ID;

            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON)
            status = oberon_cipher_encrypt_setup(
                &operation->ctx.oberon_driver_ctx,
                attributes,
                key_buffer,
                key_buffer_size,
                alg );
            /* Declared with fallback == true */
            if( status == PSA_SUCCESS )
                operation->id = PSA_CRYPTO_OBERON_DRIVER_ID;

            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
#if defined(MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT)
            /* Fell through, meaning no accelerator supports this operation */
            status = mbedtls_psa_cipher_encrypt_setup( &operation->ctx.mbedtls_ctx,
                                                       attributes,
                                                       key_buffer,
                                                       key_buffer_size,
                                                       alg );
            if( status == PSA_SUCCESS )
                operation->id = PSA_CRYPTO_MBED_TLS_DRIVER_ID;

            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT */
            return( PSA_ERROR_NOT_SUPPORTED );
        default:
            /* Key is declared with a lifetime not known to us */
            (void)status;
            (void)key_buffer;
            (void)key_buffer_size;
            (void)alg;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_cipher_decrypt_setup(
    psa_cipher_operation_t *operation,
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg )
{
    psa_status_t status = PSA_ERROR_INVALID_ARGUMENT;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX)
            status = cc3xx_cipher_decrypt_setup(
                &operation->ctx.cc3xx_driver_ctx,
                attributes,
                key_buffer,
                key_buffer_size,
                alg );
            /* Declared with fallback == true */
            if( status == PSA_SUCCESS )
                operation->id = PSA_CRYPTO_CC3XX_DRIVER_ID;

            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON)
            status = oberon_cipher_decrypt_setup(
                &operation->ctx.oberon_driver_ctx,
                attributes,
                key_buffer,
                key_buffer_size,
                alg );
            /* Declared with fallback == true */
            if( status == PSA_SUCCESS )
                operation->id = PSA_CRYPTO_OBERON_DRIVER_ID;

            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
#if defined(MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT)
            /* Fell through, meaning no accelerator supports this operation */
            status = mbedtls_psa_cipher_decrypt_setup( &operation->ctx.mbedtls_ctx,
                                                       attributes,
                                                       key_buffer,
                                                       key_buffer_size,
                                                       alg );
            if( status == PSA_SUCCESS )
                operation->id = PSA_CRYPTO_MBED_TLS_DRIVER_ID;

            return( status );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT */
            return( PSA_ERROR_NOT_SUPPORTED );
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
        default:
            /* Key is declared with a lifetime not known to us */
            (void)status;
            (void)key_buffer;
            (void)key_buffer_size;
            (void)alg;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_cipher_set_iv(
    psa_cipher_operation_t *operation,
    const uint8_t *iv,
    size_t iv_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_cipher_set_iv( &operation->ctx.mbedtls_ctx,
                                               iv,
                                               iv_length ) );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return( cc3xx_cipher_set_iv(
                        &operation->ctx.cc3xx_driver_ctx,
                        iv, iv_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            return( oberon_cipher_set_iv(
                        &operation->ctx.oberon_driver_ctx,
                        iv, iv_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
    }

    (void)iv;
    (void)iv_length;

    return( PSA_ERROR_INVALID_ARGUMENT );
}

psa_status_t psa_driver_wrapper_cipher_update(
    psa_cipher_operation_t *operation,
    const uint8_t *input,
    size_t input_length,
    uint8_t *output,
    size_t output_size,
    size_t *output_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_cipher_update( &operation->ctx.mbedtls_ctx,
                                               input,
                                               input_length,
                                               output,
                                               output_size,
                                               output_length ) );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return( cc3xx_cipher_update(
                        &operation->ctx.cc3xx_driver_ctx,
                        input, input_length,
                        output, output_size, output_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            return( oberon_cipher_update(
                        &operation->ctx.oberon_driver_ctx,
                        input, input_length,
                        output, output_size, output_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
    }

    (void)input;
    (void)input_length;
    (void)output;
    (void)output_size;
    (void)output_length;

    return( PSA_ERROR_INVALID_ARGUMENT );
}

psa_status_t psa_driver_wrapper_cipher_finish(
    psa_cipher_operation_t *operation,
    uint8_t *output,
    size_t output_size,
    size_t *output_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_cipher_finish( &operation->ctx.mbedtls_ctx,
                                               output,
                                               output_size,
                                               output_length ) );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return( cc3xx_cipher_finish(
                        &operation->ctx.cc3xx_driver_ctx,
                        output, output_size, output_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX*/
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            return( oberon_cipher_finish(
                        &operation->ctx.oberon_driver_ctx,
                        output, output_size, output_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
    }

    (void)output;
    (void)output_size;
    (void)output_length;

    return( PSA_ERROR_INVALID_ARGUMENT );
}

psa_status_t psa_driver_wrapper_cipher_abort(
    psa_cipher_operation_t *operation )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;

    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_cipher_abort( &operation->ctx.mbedtls_ctx ) );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_CIPHER_SUPPORT */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            status = cc3xx_cipher_abort(
                         &operation->ctx.cc3xx_driver_ctx );
            mbedtls_platform_zeroize(
                &operation->ctx.cc3xx_driver_ctx,
                sizeof( operation->ctx.cc3xx_driver_ctx ) );
            return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            status = oberon_cipher_abort(
                         &operation->ctx.oberon_driver_ctx );
            mbedtls_platform_zeroize(
                &operation->ctx.oberon_driver_ctx,
                sizeof( operation->ctx.oberon_driver_ctx ) );
            return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_CIPHER_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
    }

    (void)status;
    return( PSA_ERROR_INVALID_ARGUMENT );
}

/*
 * Hashing functions
 */
psa_status_t psa_driver_wrapper_hash_compute(
    psa_algorithm_t alg,
    const uint8_t *input,
    size_t input_length,
    uint8_t *hash,
    size_t hash_size,
    size_t *hash_length)
{
#if !defined(PSA_WANT_ALG_SHA_1)
    if (alg == PSA_ALG_SHA_1) {
        return PSA_ERROR_NOT_SUPPORTED;
    }
#endif

    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;

    /* Try accelerators first */
#if defined(PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_CC3XX)
    status = cc3xx_hash_compute(alg, input, input_length, hash, hash_size,
            hash_length);
    if (status != PSA_ERROR_NOT_SUPPORTED)
        return status;
#endif /* PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_OBERON)
    status = oberon_hash_compute(alg, input, input_length, hash, hash_size,
            hash_length);
    if (status != PSA_ERROR_NOT_SUPPORTED)
        return status;
#endif /* PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_OBERON */

    /* If software fallback is compiled in, try fallback */
#if defined(MBEDTLS_PSA_BUILTIN_HAS_HASH_SUPPORT)
    status = mbedtls_psa_hash_compute( alg, input, input_length,
                                       hash, hash_size, hash_length );
    if( status != PSA_ERROR_NOT_SUPPORTED )
        return( status );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_HASH_SUPPORT */
    (void) status;
    (void) alg;
    (void) input;
    (void) input_length;
    (void) hash;
    (void) hash_size;
    (void) hash_length;

    return( PSA_ERROR_NOT_SUPPORTED );
}

psa_status_t psa_driver_wrapper_hash_setup(
    psa_hash_operation_t *operation,
    psa_algorithm_t alg )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;

#if !defined(PSA_WANT_ALG_SHA_1)
    if (alg == PSA_ALG_SHA_1) {
        return PSA_ERROR_NOT_SUPPORTED;
    }
#endif

    /* Try setup on accelerators first */
#if defined(PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_CC3XX)
    status = cc3xx_hash_setup(&operation->ctx.cc3xx_driver_ctx, alg);
    if( status == PSA_SUCCESS )
        operation->id = PSA_CRYPTO_CC3XX_DRIVER_ID;

    if( status != PSA_ERROR_NOT_SUPPORTED) {
        return( status );
    }
#endif /* PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_OBERON)
    status = oberon_hash_setup(&operation->ctx.oberon_driver_ctx, alg);
    if( status == PSA_SUCCESS )
        operation->id = PSA_CRYPTO_OBERON_DRIVER_ID;

    if( status != PSA_ERROR_NOT_SUPPORTED) {
        return( status );
    }
#endif /* PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_OBERON */

    /* If software fallback is compiled in, try fallback */
#if defined(MBEDTLS_PSA_BUILTIN_HAS_HASH_SUPPORT)
    status = mbedtls_psa_hash_setup( &operation->ctx.mbedtls_ctx, alg );
    if( status == PSA_SUCCESS )
        operation->id = PSA_CRYPTO_MBED_TLS_DRIVER_ID;

    if( status != PSA_ERROR_NOT_SUPPORTED )
        return( status );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_HASH_SUPPORT */
    /* Nothing left to try if we fall through here */
    (void) status;
    (void) operation;
    (void) alg;
    return( PSA_ERROR_NOT_SUPPORTED );
}

psa_status_t psa_driver_wrapper_hash_clone(
    const psa_hash_operation_t *source_operation,
    psa_hash_operation_t *target_operation )
{
    switch( source_operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_HASH_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            target_operation->id = PSA_CRYPTO_MBED_TLS_DRIVER_ID;
            return( mbedtls_psa_hash_clone( &source_operation->ctx.mbedtls_ctx,
                                            &target_operation->ctx.mbedtls_ctx ) );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_HASH_SUPPORT */
#if defined(PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            target_operation->id = PSA_CRYPTO_CC3XX_DRIVER_ID;
            return( cc3xx_hash_clone(
                        &source_operation->ctx.cc3xx_driver_ctx,
                        &target_operation->ctx.cc3xx_driver_ctx ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            target_operation->id = PSA_CRYPTO_OBERON_DRIVER_ID;
            return( oberon_hash_clone(
                        &source_operation->ctx.oberon_driver_ctx,
                        &target_operation->ctx.oberon_driver_ctx ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_OBERON */
        default:
            (void) target_operation;
            return( PSA_ERROR_BAD_STATE );
    }
}

psa_status_t psa_driver_wrapper_hash_update(
    psa_hash_operation_t *operation,
    const uint8_t *input,
    size_t input_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_HASH_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_hash_update( &operation->ctx.mbedtls_ctx,
                                             input, input_length ) );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_HASH_SUPPORT */
#if defined(PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return( cc3xx_hash_update(
                        &operation->ctx.cc3xx_driver_ctx,
                        input, input_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            return( oberon_hash_update(
                        &operation->ctx.oberon_driver_ctx,
                        input, input_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_OBERON */
        default:
            (void) input;
            (void) input_length;
            return( PSA_ERROR_BAD_STATE );
    }
}

psa_status_t psa_driver_wrapper_hash_finish(
    psa_hash_operation_t *operation,
    uint8_t *hash,
    size_t hash_size,
    size_t *hash_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_HASH_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_hash_finish( &operation->ctx.mbedtls_ctx,
                                             hash, hash_size, hash_length ) );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_HASH_SUPPORT */
#if defined(PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return( cc3xx_hash_finish(
                        &operation->ctx.cc3xx_driver_ctx,
                        hash, hash_size, hash_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            return( oberon_hash_finish(
                        &operation->ctx.oberon_driver_ctx,
                        hash, hash_size, hash_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_OBERON */
        default:
            (void) hash;
            (void) hash_size;
            (void) hash_length;
            return( PSA_ERROR_BAD_STATE );
    }
}

psa_status_t psa_driver_wrapper_hash_abort(
    psa_hash_operation_t *operation )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_HASH_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_hash_abort( &operation->ctx.mbedtls_ctx ) );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_HASH_SUPPORT */
#if defined(PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return( cc3xx_hash_abort(
                        &operation->ctx.cc3xx_driver_ctx ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            return( oberon_hash_abort(
                        &operation->ctx.oberon_driver_ctx ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_HASH_SUPPORT_OBERON */
        default:
            return( PSA_ERROR_BAD_STATE );
    }
}

psa_status_t psa_driver_wrapper_aead_encrypt(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *nonce, size_t nonce_length,
    const uint8_t *additional_data, size_t additional_data_length,
    const uint8_t *plaintext, size_t plaintext_length,
    uint8_t *ciphertext, size_t ciphertext_size, size_t *ciphertext_length )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX)
            status = cc3xx_aead_encrypt(
                        attributes, key_buffer, key_buffer_size,
                        alg,
                        nonce, nonce_length,
                        additional_data, additional_data_length,
                        plaintext, plaintext_length,
                        ciphertext, ciphertext_size, ciphertext_length );

            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON)
            status = oberon_aead_encrypt(
                        attributes, key_buffer, key_buffer_size,
                        alg,
                        nonce, nonce_length,
                        additional_data, additional_data_length,
                        plaintext, plaintext_length,
                        ciphertext, ciphertext_size, ciphertext_length );

            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
#if defined(MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT)
            /* Fell through, meaning no accelerator supports this operation */
            status =( mbedtls_psa_aead_encrypt(
                        attributes, key_buffer, key_buffer_size,
                        alg,
                        nonce, nonce_length,
                        additional_data, additional_data_length,
                        plaintext, plaintext_length,
                        ciphertext, ciphertext_size, ciphertext_length ) );
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT */
            (void)attributes;
            (void)key_buffer;
            (void)key_buffer_size;
            (void)alg;
            (void)nonce;
            (void)nonce_length;
            (void)additional_data;
            (void)additional_data_length;
            (void)plaintext;
            (void)plaintext_length;
            (void)ciphertext;
            (void)ciphertext_size;
            return( PSA_ERROR_NOT_SUPPORTED );
        default:
            /* Key is declared with a lifetime not known to us */
            (void)status;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_aead_decrypt(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer, size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *nonce, size_t nonce_length,
    const uint8_t *additional_data, size_t additional_data_length,
    const uint8_t *ciphertext, size_t ciphertext_length,
    uint8_t *plaintext, size_t plaintext_size, size_t *plaintext_length )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX)
            status = cc3xx_aead_decrypt(
                        attributes, key_buffer, key_buffer_size,
                        alg,
                        nonce, nonce_length,
                        additional_data, additional_data_length,
                        ciphertext, ciphertext_length,
                        plaintext, plaintext_size, plaintext_length );

            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON)
            status = oberon_aead_decrypt(
                        attributes, key_buffer, key_buffer_size,
                        alg,
                        nonce, nonce_length,
                        additional_data, additional_data_length,
                        ciphertext, ciphertext_length,
                        plaintext, plaintext_size, plaintext_length );

            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */

#if defined(MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT)
            /* Fell through, meaning no accelerator supports this operation */
            status = mbedtls_psa_aead_decrypt(
                        attributes, key_buffer, key_buffer_size,
                        alg,
                        nonce, nonce_length,
                        additional_data, additional_data_length,
                        ciphertext, ciphertext_length,
                        plaintext, plaintext_size, plaintext_length );
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT */

            (void)attributes;
            (void)attributes;
            (void)key_buffer;
            (void)key_buffer_size;
            (void)alg;
            (void)nonce;
            (void)nonce_length;
            (void)additional_data;
            (void)additional_data_length;
            (void)ciphertext;
            (void)ciphertext_length;
            (void)plaintext;
            (void)plaintext_size;
            (void)plaintext_length;
            return( PSA_ERROR_NOT_SUPPORTED );
        default:
            /* Key is declared with a lifetime not known to us */
            (void)status;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_aead_encrypt_setup(
   psa_aead_operation_t *operation,
   const psa_key_attributes_t *attributes,
   const uint8_t *key_buffer, size_t key_buffer_size,
   psa_algorithm_t alg )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX)
            operation->id = PSA_CRYPTO_CC3XX_DRIVER_ID;
            status = cc3xx_aead_encrypt_setup(
                    &operation->ctx.cc3xx_driver_ctx,
                    attributes, key_buffer, key_buffer_size,
                    alg );

            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON)
            operation->id = PSA_CRYPTO_OBERON_DRIVER_ID;
            status = oberon_aead_encrypt_setup(
                    &operation->ctx.oberon_driver_ctx,
                    attributes, key_buffer, key_buffer_size,
                    alg );

            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON*/
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */


#if defined(MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT)
            /* Fell through, meaning no accelerator supports this operation */
            operation->id = PSA_CRYPTO_MBED_TLS_DRIVER_ID;
            status = mbedtls_psa_aead_encrypt_setup(
                        &operation->ctx.mbedtls_ctx, attributes,
                        key_buffer, key_buffer_size,
                        alg );

            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT*/

            (void)operation;
            (void)attributes;
            (void)key_buffer;
            (void)key_buffer_size;
            (void)alg;
            return( PSA_ERROR_NOT_SUPPORTED );
        default:
            /* Key is declared with a lifetime not known to us */
            (void)status;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_aead_decrypt_setup(
   psa_aead_operation_t *operation,
   const psa_key_attributes_t *attributes,
   const uint8_t *key_buffer, size_t key_buffer_size,
   psa_algorithm_t alg )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX)
            operation->id = PSA_CRYPTO_CC3XX_DRIVER_ID;
            status = cc3xx_aead_decrypt_setup(
                    &operation->ctx.cc3xx_driver_ctx,
                    attributes,
                    key_buffer, key_buffer_size,
                    alg );

            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX  */
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON)
            operation->id = PSA_CRYPTO_OBERON_DRIVER_ID;
            status = oberon_aead_decrypt_setup(
                    &operation->ctx.oberon_driver_ctx,
                    attributes,
                    key_buffer, key_buffer_size,
                    alg );

            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */

#if defined(MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT)
            /* Fell through, meaning no accelerator supports this operation */
            operation->id = PSA_CRYPTO_MBED_TLS_DRIVER_ID;
            status = mbedtls_psa_aead_decrypt_setup(
                        &operation->ctx.mbedtls_ctx,
                        attributes,
                        key_buffer, key_buffer_size,
                        alg );

            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT */

            (void)operation;
            (void)attributes;
            (void)key_buffer;
            (void)key_buffer_size;
            (void)alg;
            return( PSA_ERROR_NOT_SUPPORTED );
        default:
            /* Key is declared with a lifetime not known to us */
            (void)status;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_aead_set_nonce(
   psa_aead_operation_t *operation,
   const uint8_t *nonce,
   size_t nonce_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_aead_set_nonce( &operation->ctx.mbedtls_ctx,
                                                nonce,
                                                nonce_length ) );

#endif /* MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return( cc3xx_aead_set_nonce(
                    &operation->ctx.cc3xx_driver_ctx,
                    nonce, nonce_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            return( oberon_aead_set_nonce(
                    &operation->ctx.oberon_driver_ctx,
                    nonce, nonce_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
    }

    (void)nonce;
    (void)nonce_length;

    return( PSA_ERROR_INVALID_ARGUMENT );
}

psa_status_t psa_driver_wrapper_aead_set_lengths(
   psa_aead_operation_t *operation,
   size_t ad_length,
   size_t plaintext_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_aead_set_lengths( &operation->ctx.mbedtls_ctx,
                                                  ad_length,
                                                  plaintext_length ) );

#endif /* MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return( cc3xx_aead_set_lengths(
                    &operation->ctx.cc3xx_driver_ctx,
                    ad_length, plaintext_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            return( oberon_aead_set_lengths(
                    &operation->ctx.oberon_driver_ctx,
                    ad_length, plaintext_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
    }

    (void)ad_length;
    (void)plaintext_length;

    return( PSA_ERROR_INVALID_ARGUMENT );
}

psa_status_t psa_driver_wrapper_aead_update_ad(
   psa_aead_operation_t *operation,
   const uint8_t *input,
   size_t input_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_aead_update_ad( &operation->ctx.mbedtls_ctx,
                                                input,
                                                input_length ) );

#endif /* MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return( cc3xx_aead_update_ad(
                    &operation->ctx.cc3xx_driver_ctx,
                    input, input_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            return( oberon_aead_update_ad(
                    &operation->ctx.oberon_driver_ctx,
                    input, input_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
        default:
            (void)input;
            (void)input_length;

            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_aead_update(
   psa_aead_operation_t *operation,
   const uint8_t *input,
   size_t input_length,
   uint8_t *output,
   size_t output_size,
   size_t *output_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_aead_update( &operation->ctx.mbedtls_ctx,
                                             input, input_length,
                                             output, output_size,
                                             output_length ) );

#endif /* MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return( cc3xx_aead_update(
                    &operation->ctx.cc3xx_driver_ctx,
                    input, input_length, output, output_size,
                    output_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            return( oberon_aead_update(
                    &operation->ctx.oberon_driver_ctx,
                    input, input_length, output, output_size,
                    output_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */

        default:
        (void)input;
        (void)input_length;
        (void)output;
        (void)output_size;
        (void)output_length;

        return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_aead_finish(
   psa_aead_operation_t *operation,
   uint8_t *ciphertext,
   size_t ciphertext_size,
   size_t *ciphertext_length,
   uint8_t *tag,
   size_t tag_size,
   size_t *tag_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_aead_finish( &operation->ctx.mbedtls_ctx,
                                             ciphertext,
                                             ciphertext_size,
                                             ciphertext_length, tag,
                                             tag_size, tag_length ) );

#endif /* MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return( cc3xx_aead_finish(
                    &operation->ctx.cc3xx_driver_ctx,
                    ciphertext, ciphertext_size,
                    ciphertext_length, tag, tag_size, tag_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            return( oberon_aead_finish(
                    &operation->ctx.oberon_driver_ctx,
                    ciphertext, ciphertext_size,
                    ciphertext_length, tag, tag_size, tag_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */

        default:
            (void)ciphertext;
            (void)ciphertext_size;
            (void)ciphertext_length;
            (void)tag;
            (void)tag_size;
            (void)tag_length;

            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_aead_verify(
   psa_aead_operation_t *operation,
   uint8_t *plaintext,
   size_t plaintext_size,
   size_t *plaintext_length,
   const uint8_t *tag,
   size_t tag_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            {
                psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
                uint8_t check_tag[PSA_AEAD_TAG_MAX_SIZE];
                size_t check_tag_length;

                status = mbedtls_psa_aead_finish( &operation->ctx.mbedtls_ctx,
                                                  plaintext,
                                                  plaintext_size,
                                                  plaintext_length,
                                                  check_tag,
                                                  sizeof( check_tag ),
                                                  &check_tag_length );

                if( status == PSA_SUCCESS )
                {
                    if( tag_length != check_tag_length ||
                        mbedtls_psa_safer_memcmp( tag, check_tag, tag_length )
                        != 0 )
                        status = PSA_ERROR_INVALID_SIGNATURE;
                }

                mbedtls_platform_zeroize( check_tag, sizeof( check_tag ) );

                return( status );
            }

#endif /* MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return( cc3xx_aead_verify(
                    &operation->ctx.cc3xx_driver_ctx,
                    plaintext, plaintext_size,
                    plaintext_length, tag, tag_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON)
        case PSA_CRYPTO_OBERON_DRIVER_ID:
            return( oberon_aead_verify(
                    &operation->ctx.oberon_driver_ctx,
                    plaintext, plaintext_size,
                    plaintext_length, tag, tag_length ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */

        default:
            (void)plaintext;
            (void)plaintext_size;
            (void)plaintext_length;
            (void)tag;
            (void)tag_length;

            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_aead_abort(
   psa_aead_operation_t *operation )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_aead_abort( &operation->ctx.mbedtls_ctx ) );

#endif /* MBEDTLS_PSA_BUILTIN_HAS_AEAD_SUPPORT */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX)
    case PSA_CRYPTO_CC3XX_DRIVER_ID:
        return( cc3xx_aead_abort(
                &operation->ctx.cc3xx_driver_ctx ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON)
    case PSA_CRYPTO_OBERON_DRIVER_ID:
        return( oberon_aead_abort(
                &operation->ctx.oberon_driver_ctx ) );
#endif /* PSA_CRYPTO_DRIVER_HAS_AEAD_SUPPORT_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */

        default:
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

/*
 * MAC functions
 */
psa_status_t psa_driver_wrapper_mac_compute(
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer,
    size_t key_buffer_size,
    psa_algorithm_t alg,
    const uint8_t *input,
    size_t input_length,
    uint8_t *mac,
    size_t mac_size,
    size_t *mac_length )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

#if !defined(PSA_WANT_ALG_SHA_1)
    if (PSA_ALG_HMAC_GET_HASH(alg) == PSA_ALG_SHA_1) {
        return PSA_ERROR_NOT_SUPPORTED;
    }
#endif

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX)
            status = cc3xx_mac_compute(attributes, key_buffer, key_buffer_size, alg,
                input, input_length,
                mac, mac_size, mac_length);
            /* Declared with fallback == true */
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
#if defined(MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT)
            /* Fell through, meaning no accelerator supports this operation */
            status = mbedtls_psa_mac_compute(
                attributes, key_buffer, key_buffer_size, alg,
                input, input_length,
                mac, mac_size, mac_length );
            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
            return( PSA_ERROR_NOT_SUPPORTED );
        default:
            /* Key is declared with a lifetime not known to us */
            (void) key_buffer;
            (void) key_buffer_size;
            (void) alg;
            (void) input;
            (void) input_length;
            (void) mac;
            (void) mac_size;
            (void) mac_length;
            (void) status;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_mac_sign_setup(
    psa_mac_operation_t *operation,
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer,
    size_t key_buffer_size,
    psa_algorithm_t alg )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

#if !defined(PSA_WANT_ALG_SHA_1)
    if (PSA_ALG_HMAC_GET_HASH(alg) == PSA_ALG_SHA_1) {
        return PSA_ERROR_NOT_SUPPORTED;
    }
#endif

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX)
            status = cc3xx_mac_sign_setup(
                    &operation->ctx.cc3xx_driver_ctx,
                    attributes,
                    key_buffer, key_buffer_size,
                    alg);
            if (status == PSA_SUCCESS)
                operation->id = PSA_CRYPTO_CC3XX_DRIVER_ID;
            if (status != PSA_ERROR_NOT_SUPPORTED)
                return status;
#endif /* PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
            /* Fell through, meaning no accelerator supports this operation */
#if defined(MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT)
            status = mbedtls_psa_mac_sign_setup( &operation->ctx.mbedtls_ctx,
                                                 attributes,
                                                 key_buffer, key_buffer_size,
                                                 alg );
            if( status == PSA_SUCCESS )
                operation->id = PSA_CRYPTO_MBED_TLS_DRIVER_ID;

            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT */
            return( PSA_ERROR_NOT_SUPPORTED );
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
        default:
            /* Key is declared with a lifetime not known to us */
            (void) status;
            (void) key_buffer;
            (void) key_buffer_size;
            (void) alg;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_mac_verify_setup(
    psa_mac_operation_t *operation,
    const psa_key_attributes_t *attributes,
    const uint8_t *key_buffer,
    size_t key_buffer_size,
    psa_algorithm_t alg )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;
    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

#if !defined(PSA_WANT_ALG_SHA_1)
    if (PSA_ALG_HMAC_GET_HASH(alg) == PSA_ALG_SHA_1) {
        return PSA_ERROR_NOT_SUPPORTED;
    }
#endif

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX)
            status = cc3xx_mac_verify_setup(
                &operation->ctx.cc3xx_driver_ctx,
                attributes,
                key_buffer, key_buffer_size,
                alg);
            if (status == PSA_SUCCESS)
                operation->id = PSA_CRYPTO_CC3XX_DRIVER_ID;
            if (status != PSA_ERROR_NOT_SUPPORTED)
                return status;
#endif /* PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
#if defined(MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT)
            /* Fell through, meaning no accelerator supports this operation */
            status = mbedtls_psa_mac_verify_setup( &operation->ctx.mbedtls_ctx,
                                                   attributes,
                                                   key_buffer, key_buffer_size,
                                                   alg );
            if( status == PSA_SUCCESS )
                operation->id = PSA_CRYPTO_MBED_TLS_DRIVER_ID;

            if( status != PSA_ERROR_NOT_SUPPORTED )
                return( status );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT */
            (void) status;
            (void) key_buffer;
            (void) key_buffer_size;
            (void) alg;
            return( PSA_ERROR_NOT_SUPPORTED );
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
        default:
            /* Key is declared with a lifetime not known to us */
            (void) status;
            (void) key_buffer;
            (void) key_buffer_size;
            (void) alg;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_mac_update(
    psa_mac_operation_t *operation,
    const uint8_t *input,
    size_t input_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_mac_update( &operation->ctx.mbedtls_ctx,
                                            input, input_length ) );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return(cc3xx_mac_update(&operation->ctx.cc3xx_driver_ctx, input, input_length));
#endif /* PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
        default:
            (void) input;
            (void) input_length;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_mac_sign_finish(
    psa_mac_operation_t *operation,
    uint8_t *mac,
    size_t mac_size,
    size_t *mac_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_mac_sign_finish( &operation->ctx.mbedtls_ctx,
                                                 mac, mac_size, mac_length ) );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return(cc3xx_mac_sign_finish(&operation->ctx.cc3xx_driver_ctx,
                        mac, mac_size, mac_length));
#endif /* PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
        default:
            (void) mac;
            (void) mac_size;
            (void) mac_length;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_mac_verify_finish(
    psa_mac_operation_t *operation,
    const uint8_t *mac,
    size_t mac_length )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_mac_verify_finish( &operation->ctx.mbedtls_ctx,
                                                   mac, mac_length ) );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return(cc3xx_mac_verify_finish(
                        &operation->ctx.cc3xx_driver_ctx,
                        mac, mac_length));
#endif /* PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
        default:
            (void) mac;
            (void) mac_length;
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_mac_abort(
    psa_mac_operation_t *operation )
{
    switch( operation->id )
    {
#if defined(MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT)
        case PSA_CRYPTO_MBED_TLS_DRIVER_ID:
            return( mbedtls_psa_mac_abort( &operation->ctx.mbedtls_ctx ) );
#endif /* MBEDTLS_PSA_BUILTIN_HAS_MAC_SUPPORT */

#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX)
        case PSA_CRYPTO_CC3XX_DRIVER_ID:
            return(cc3xx_mac_abort(&operation->ctx.cc3xx_driver_ctx));
#endif /* PSA_CRYPTO_DRIVER_HAS_MAC_SUPPORT_CC3XX */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
        default:
            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

/*
 * Key agreement functions
 */
psa_status_t psa_driver_wrapper_key_agreement(
        const psa_key_attributes_t *attributes,
        const uint8_t *priv_key, size_t priv_key_size,
        const uint8_t *publ_key, size_t publ_key_size,
        uint8_t *output, size_t output_size, size_t *output_length,
        psa_algorithm_t alg )
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;

    psa_key_location_t location =
            PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_ALG_ECDH_CC3XX)
            status = cc3xx_key_agreement( attributes,
                                          priv_key,
                                          priv_key_size,
                                          publ_key,
                                          publ_key_size,
                                          output,
                                          output_size,
                                          output_length,
                                          alg );
            return( status );
#endif /* PSA_CRYPTO_DRIVER_ALG_ECDH_CC3XX */
#if defined(PSA_CRYPTO_DRIVER_ALG_ECDH_OBERON)
            status = oberon_key_agreement(attributes,
                                          priv_key,
                                          priv_key_size,
                                          publ_key,
                                          publ_key_size,
                                          output,
                                          output_size,
                                          output_length,
                                          alg);
            return( status );
#endif /* PSA_CRYPTO_DRIVER_ALG_ECDH_OBERON */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
            (void) status;
            return ( PSA_ERROR_NOT_SUPPORTED );
    default:
        /* Key is declared with a lifetime not known to us */
        (void) priv_key;
        (void) priv_key_size;
        (void) publ_key;
        (void) publ_key_size;
        (void) output;
        (void) output_size;
        (void) output_length;
        (void) alg;

        return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

/*
 * Asymmetric operations
 */
psa_status_t psa_driver_wrapper_asymmetric_encrypt(const psa_key_attributes_t *attributes,
                                const uint8_t *key_buffer,
                                size_t key_buffer_size, psa_algorithm_t alg,
                                const uint8_t *input, size_t input_length,
                                const uint8_t *salt, size_t salt_length,
                                uint8_t *output, size_t output_size,
                                size_t *output_length)
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;

    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_ASYM_ENCRYPT_SUPPORT_CC3XX)
            status = cc3xx_asymmetric_encrypt( attributes,
                                               key_buffer,
                                               key_buffer_size,
                                               alg,
                                               input,
                                               input_length,
                                               salt,
                                               salt_length,
                                               output,
                                               output_size,
                                               output_length );
            return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ASYM_ENCRYPT_SUPPORT_CC3XX */
#endif  /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
            (void) status;
            return ( PSA_ERROR_NOT_SUPPORTED );
        default:
            /* Key is declared with a lifetime not known to us */
            (void) key_buffer;
            (void) key_buffer_size;
            (void) alg;
            (void) input;
            (void) input_length;
            (void) salt;
            (void) salt_length;
            (void) output;
            (void) output_size;
            (void) output_length;

            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

psa_status_t psa_driver_wrapper_asymmetric_decrypt(const psa_key_attributes_t *attributes,
                                const uint8_t *key_buffer,
                                size_t key_buffer_size, psa_algorithm_t alg,
                                const uint8_t *input, size_t input_length,
                                const uint8_t *salt, size_t salt_length,
                                uint8_t *output, size_t output_size,
                                size_t *output_length)
{
    psa_status_t status = PSA_ERROR_CORRUPTION_DETECTED;

    psa_key_location_t location =
        PSA_KEY_LIFETIME_GET_LOCATION( attributes->core.lifetime );

    switch( location )
    {
        case PSA_KEY_LOCATION_LOCAL_STORAGE:
            /* Key is stored in the slot in export representation, so
             * cycle through all known transparent accelerators */
#if defined(PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT)
#if defined(PSA_CRYPTO_DRIVER_HAS_ASYM_ENCRYPT_SUPPORT_CC3XX)
            status = cc3xx_asymmetric_decrypt( attributes,
                                               key_buffer,
                                               key_buffer_size,
                                               alg,
                                               input,
                                               input_length,
                                               salt,
                                               salt_length,
                                               output,
                                               output_size,
                                               output_length );
            return( status );
#endif /* PSA_CRYPTO_DRIVER_HAS_ASYM_ENCRYPT_SUPPORT_CC3XX */
#endif /* PSA_CRYPTO_ACCELERATOR_DRIVER_PRESENT */
            (void) status;
            return( PSA_ERROR_NOT_SUPPORTED );
        default:
            /* Key is declared with a lifetime not known to us */
            (void) key_buffer;
            (void) key_buffer_size;
            (void) alg;
            (void) input;
            (void) input_length;
            (void) salt;
            (void) salt_length;
            (void) output;
            (void) output_size;
            (void) output_length;

            return( PSA_ERROR_INVALID_ARGUMENT );
    }
}

#endif /* MBEDTLS_PSA_CRYPTO_C */
