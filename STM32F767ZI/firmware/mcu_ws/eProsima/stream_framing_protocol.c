#include <uxr/client/util/time.h>
#include "stream_framing_protocol.h"
#include <string.h>
// Replace with the correct absolute path
#include "~/firmware/freertos_apps/exp.h"

/*******************************************************************************
* Static members.
*******************************************************************************/
// CRC-16 table for POLY 0x8005 (x^16 + x^15 + x^2 + 1).
static const uint16_t crc16_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

/*******************************************************************************
* Public function definitions.
*******************************************************************************/
void uxr_update_crc(
        uint16_t* crc,
        const uint8_t data)
{
    *crc = (*crc >> 8) ^ crc16_table[(*crc ^ data) & 0xFF];
}

bool uxr_get_next_octet(
        uxrFramingIO* framing_io,
        uint8_t* octet)
{
    bool rv = false;
    *octet = 0;
    if (framing_io->rb_head != framing_io->rb_tail)
    {
        if (UXR_FRAMING_ESC_FLAG != framing_io->rb[framing_io->rb_tail])
        {
            *octet = framing_io->rb[framing_io->rb_tail];
            framing_io->rb_tail = (uint8_t)((size_t)(framing_io->rb_tail + 1) % sizeof(framing_io->rb));
            rv = (UXR_FRAMING_BEGIN_FLAG != *octet);
        }
        else
        {
            uint8_t temp_tail = (uint8_t)((size_t)(framing_io->rb_tail + 1) % sizeof(framing_io->rb));
            if (temp_tail != framing_io->rb_head)
            {
                *octet = framing_io->rb[temp_tail];
                framing_io->rb_tail = (uint8_t)((size_t)(framing_io->rb_tail + 2) % sizeof(framing_io->rb));
                if (UXR_FRAMING_BEGIN_FLAG != *octet)
                {
                    *octet ^= UXR_FRAMING_XOR_FLAG;
                    rv = true;
                }
            }
        }
    }
    return rv;
}

bool uxr_add_next_octet(
        uxrFramingIO* framing_io,
        uint8_t octet)
{
    bool rv = false;

    if (UXR_FRAMING_BEGIN_FLAG == octet || UXR_FRAMING_ESC_FLAG == octet)
    {
        if ((uint8_t)(framing_io->wb_pos + 1) < sizeof(framing_io->wb))
        {
            framing_io->wb[framing_io->wb_pos] = UXR_FRAMING_ESC_FLAG;
            framing_io->wb[framing_io->wb_pos + 1] = octet ^ UXR_FRAMING_XOR_FLAG;
            framing_io->wb_pos = (uint8_t)(framing_io->wb_pos + 2);
            rv = true;
        }
    }
    else
    {
        if (framing_io->wb_pos < sizeof(framing_io->wb))
        {
            framing_io->wb[framing_io->wb_pos] = octet;
            framing_io->wb_pos = (uint8_t)(framing_io->wb_pos + 1);
            rv = true;
        }
    }

    return rv;
}

bool uxr_add_next_octet_new(
        uxrFramingIO* framing_io,
        uint8_t octet)
{
    bool rv = false;

    if (UXR_FRAMING_BEGIN_FLAG == octet || UXR_FRAMING_ESC_FLAG == octet)
    {
        if ((framing_io->wb_pos + 1) < sizeof(framing_io->wb))
        {
            framing_io->wb[framing_io->wb_pos] = UXR_FRAMING_ESC_FLAG;
            framing_io->wb[framing_io->wb_pos + 1] = octet ^ UXR_FRAMING_XOR_FLAG;
            framing_io->wb_pos = (framing_io->wb_pos + 2);
            rv = true;
        }
    }
    else
    {
        if (framing_io->wb_pos < sizeof(framing_io->wb))
        {
            framing_io->wb[framing_io->wb_pos] = octet;
            framing_io->wb_pos = (framing_io->wb_pos + 1);
            rv = true;
        }
    }

    return rv;
}

void uxr_init_framing_io(
        uxrFramingIO* framing_io,
        uint8_t local_addr)
{
    framing_io->local_addr = local_addr;
    framing_io->state = UXR_FRAMING_UNINITIALIZED;
    framing_io->rb_head = 0;
    framing_io->rb_tail = 0;
}

bool uxr_framing_write_transport(
        uxrFramingIO* framing_io,
        uxr_write_cb write_cb,
        void* cb_arg,
        uint8_t* errcode)
{
    size_t bytes_written = 0;
    size_t last_written = 0;

    do
    {
        last_written = write_cb(cb_arg, &framing_io->wb[bytes_written], framing_io->wb_pos - bytes_written, errcode);
        bytes_written += last_written;
    } while (bytes_written < framing_io->wb_pos && 0 < last_written);

    if (bytes_written == framing_io->wb_pos)
    {
        framing_io->wb_pos = 0;
        return true;
    }
    return false;
}

bool uxr_framing_write_transport_new(
        uxrFramingIO* framing_io,
        uxr_write_cb_new write_cb_new,
        void* cb_arg,
        uint8_t* errcode,
        int priority,
        uint8_t QoS)
{
    size_t bytes_written = 0;
    size_t last_written = 0;

    do
    {
        last_written = write_cb_new(cb_arg, &framing_io->wb[bytes_written], framing_io->wb_pos - bytes_written, errcode, priority, QoS);
        bytes_written += last_written;
    } while (bytes_written < framing_io->wb_pos && 0 < last_written);

    if (bytes_written == framing_io->wb_pos)
    {
        framing_io->wb_pos = 0;
        return true;
    }
    return false;
}

size_t uxr_write_framed_msg(
        uxrFramingIO* framing_io,
        uxr_write_cb write_cb,
        void* cb_arg,
        const uint8_t* buf,
        size_t len,
        uint8_t remote_addr,
        uint8_t* errcode)
{
    /* Buffer being flag. */
    framing_io->wb[0] = UXR_FRAMING_BEGIN_FLAG;
    framing_io->wb_pos = 1;

    /* Buffer header. */
    uxr_add_next_octet_new(framing_io, framing_io->local_addr);
    uxr_add_next_octet_new(framing_io, remote_addr);
    uxr_add_next_octet_new(framing_io, (uint8_t)(len & 0xFF));
    uxr_add_next_octet_new(framing_io, (uint8_t)(len >> 8));

    int i = 0;
    /* Write payload. */
    uint8_t octet = 0;
    size_t written_len = 0;
    uint16_t crc = 0;
    bool cond = true;
    while ((written_len < len) && cond)
    {
        octet = *(buf + written_len);
        if (uxr_add_next_octet_new(framing_io, octet))
        {
            uxr_update_crc(&crc, octet);
            ++written_len;
        }
        else
        {
            cond = uxr_framing_write_transport(framing_io, write_cb, cb_arg, errcode);
        }
    }

    /* Write CRC. */
    uint8_t tmp_crc[2];
    tmp_crc[0] = (uint8_t)(crc & 0xFF);
    tmp_crc[1] = (uint8_t)(crc >> 8);
    written_len = 0;
    while (written_len < sizeof(tmp_crc) && cond)
    {
        octet = *(tmp_crc + written_len);
        if (uxr_add_next_octet_new(framing_io, octet))
        {
            uxr_update_crc(&crc, octet);
            ++written_len;
        }
        else
        {
            cond = uxr_framing_write_transport(framing_io, write_cb, cb_arg, errcode);
        }
    }

    /* Flush write buffer. */
    if (cond && (0 < framing_io->wb_pos))
    {
        cond = uxr_framing_write_transport(framing_io, write_cb, cb_arg, errcode);
    }
    return cond ? (uint16_t)(len) : 0;
}

size_t uxr_write_framed_msg_new(
        uxrFramingIO* framing_io,
        uxr_write_cb_new write_cb_new,
        void* cb_arg,
        const uint8_t* buf,
        size_t len,
        uint8_t remote_addr,
        uint8_t* errcode,
        int priority,
        uint8_t QoS)
{
    /* Buffer being flag. */
    framing_io->wb[0] = UXR_FRAMING_BEGIN_FLAG;
    framing_io->wb_pos = 1;

    /* Buffer header. */
    uxr_add_next_octet_new(framing_io, framing_io->local_addr);
    uxr_add_next_octet_new(framing_io, remote_addr);
    uxr_add_next_octet_new(framing_io, (uint8_t)(len & 0xFF));
    uxr_add_next_octet_new(framing_io, (uint8_t)(len >> 8));

    /* Write payload. */
    uint8_t octet = 0;
    size_t written_len = 0;
    uint16_t crc = 0;
    bool cond = true;
    while ((written_len < len) && cond)
    {
        octet = *(buf + written_len);
        if (uxr_add_next_octet_new(framing_io, octet))
        {
            uxr_update_crc(&crc, octet);
            ++written_len;
        }
        else
        {
            cond = uxr_framing_write_transport_new(framing_io, write_cb_new, cb_arg, errcode, priority, QoS);
        }
    }

    /* Write CRC. */
    uint8_t tmp_crc[2];
    tmp_crc[0] = (uint8_t)(crc & 0xFF);
    tmp_crc[1] = (uint8_t)(crc >> 8);
    written_len = 0;
    while (written_len < sizeof(tmp_crc) && cond)
    {
        octet = *(tmp_crc + written_len);
        if (uxr_add_next_octet_new(framing_io, octet))
        {
            uxr_update_crc(&crc, octet);
            ++written_len;
        }
        else
        {
            cond = uxr_framing_write_transport_new(framing_io, write_cb_new, cb_arg, errcode, priority, QoS);
        }
    }


    /* Flush write buffer. */
    if (cond && (0 < framing_io->wb_pos))
    {
        cond = uxr_framing_write_transport_new(framing_io, write_cb_new, cb_arg, errcode, priority, QoS);
    }
    return cond ? (uint16_t)(len) : 0;
}

size_t uxr_framing_read_transport(
        uxrFramingIO* framing_io,
        uxr_read_cb read_cb,
        void* cb_arg,
        int* timeout,
        uint8_t* errcode,
        size_t max_size)
{
    int64_t time_init = uxr_millis();

    /* Compute read-buffer available size. */
    uint8_t av_len[2] = {
        0, 0
    };
    if (framing_io->rb_head == framing_io->rb_tail)
    {
        framing_io->rb_head = 0;
        framing_io->rb_tail = 0;
        av_len[0] = sizeof(framing_io->rb) - 1;
    }
    else if (framing_io->rb_head > framing_io->rb_tail)
    {
        if (0 < framing_io->rb_tail)
        {
            av_len[0] = (uint8_t)(sizeof(framing_io->rb) - framing_io->rb_head);
            av_len[1] = (uint8_t)(framing_io->rb_tail - 1);
        }
        else
        {
            av_len[0] = (uint8_t)(sizeof(framing_io->rb) - framing_io->rb_head - 1);
        }
    }
    else
    {
        av_len[0] = (uint8_t)(framing_io->rb_tail - framing_io->rb_head - 1);
    }

    /* Read */
    size_t bytes_read[2] = {
        0
    };

    // Limit the reading size
    if (max_size < av_len[0])
    {
        av_len[0] = (uint8_t)max_size;
        av_len[1] = 0;
    }
    else if (max_size < av_len[0] + av_len[1])
    {
        av_len[1] = (uint8_t)(max_size - av_len[0]);
    }

    if (0 < av_len[0])
    {
        bytes_read[0] = read_cb(cb_arg, &framing_io->rb[framing_io->rb_head], av_len[0], *timeout, errcode);
        framing_io->rb_head = (uint8_t)((size_t)(framing_io->rb_head + bytes_read[0]) % sizeof(framing_io->rb));
        if (0 < bytes_read[0])
        {
            if ((bytes_read[0] == av_len[0]) && (0 < av_len[1]))
            {
                bytes_read[1] = read_cb(cb_arg, &framing_io->rb[framing_io->rb_head], av_len[1], 0, errcode);
                framing_io->rb_head = (uint8_t)((size_t)(framing_io->rb_head + bytes_read[1]) % sizeof(framing_io->rb));
            }
        }
    }

    *timeout -= (int)(uxr_millis() - time_init);
    *timeout = (0 > *timeout) ? 0 : *timeout;
    return bytes_read[0] + bytes_read[1];
}

size_t uxr_read_framed_msg(
        uxrFramingIO* framing_io,
        uxr_read_cb read_cb,
        void* cb_arg,
        uint8_t* buf,
        size_t len,
        uint8_t* remote_addr,
        int* timeout,
        uint8_t* errcode)
{
    size_t rv = 0;

    if (framing_io->rb_head == framing_io->rb_tail)
    {
        uxr_framing_read_transport(framing_io, read_cb, cb_arg, timeout, errcode, 5);
    }

    if (framing_io->rb_tail != framing_io->rb_head)
    {
        /* State Machine. */
        bool exit_cond = false;
        while (!exit_cond)
        {
            uint8_t octet = 0;
            switch (framing_io->state)
            {
                case UXR_FRAMING_UNINITIALIZED:
                {
                    octet = 0;
                    while ((UXR_FRAMING_BEGIN_FLAG != octet) && (framing_io->rb_head != framing_io->rb_tail))
                    {
                        octet = framing_io->rb[framing_io->rb_tail];
                        framing_io->rb_tail = (uint8_t)((size_t)(framing_io->rb_tail + 1) % sizeof(framing_io->rb));
                    }

                    if (UXR_FRAMING_BEGIN_FLAG == octet)
                    {
                        framing_io->state = UXR_FRAMING_READING_SRC_ADDR;
                    }
                    else
                    {
                        exit_cond = true;
                    }
                    break;
                }
                case UXR_FRAMING_READING_SRC_ADDR:
                {
                    if (uxr_get_next_octet(framing_io, &framing_io->src_addr))
                    {
                        framing_io->state = UXR_FRAMING_READING_DST_ADDR;
                    }
                    else if (0 < uxr_framing_read_transport(framing_io, read_cb, cb_arg, timeout, errcode, 4))
                    {

                    }
                    else
                    {
                        if (UXR_FRAMING_BEGIN_FLAG != framing_io->src_addr)
                        {
                            exit_cond = true;
                        }
                    }
                    break;
                }
                case UXR_FRAMING_READING_DST_ADDR:
                {
                    if (uxr_get_next_octet(framing_io, &octet))
                    {
                        framing_io->state = (octet == framing_io->local_addr) ? UXR_FRAMING_READING_LEN_LSB :
                                UXR_FRAMING_UNINITIALIZED;
                    }
                    else if (0 < uxr_framing_read_transport(framing_io, read_cb, cb_arg, timeout, errcode, 3))
                    {

                    }
                    else
                    {
                        if (UXR_FRAMING_BEGIN_FLAG == octet)
                        {
                            framing_io->state = UXR_FRAMING_READING_SRC_ADDR;
                        }
                        else
                        {
                            exit_cond = true;
                        }
                    }
                    break;
                }
                case UXR_FRAMING_READING_LEN_LSB:
                {
                    if (uxr_get_next_octet(framing_io, &octet))
                    {
                        framing_io->msg_len = octet;
                        framing_io->state = UXR_FRAMING_READING_LEN_MSB;
                    }
                    else if (0 < uxr_framing_read_transport(framing_io, read_cb, cb_arg, timeout, errcode, 2))
                    {

                    }
                    else
                    {
                        if (UXR_FRAMING_BEGIN_FLAG == octet)
                        {
                            framing_io->state = UXR_FRAMING_READING_SRC_ADDR;
                        }
                        else
                        {
                            exit_cond = true;
                        }
                    }
                    break;
                }
                case UXR_FRAMING_READING_LEN_MSB:
                {
                    if (uxr_get_next_octet(framing_io, &octet))
                    {
                        framing_io->msg_len = (uint16_t)(framing_io->msg_len + (octet << 8));
                        framing_io->msg_pos = 0;
                        framing_io->cmp_crc = 0;
                        if (len < framing_io->msg_len)
                        {
                            framing_io->state = UXR_FRAMING_UNINITIALIZED;
                            exit_cond = true;
                        }
                        else
                        {
                            framing_io->state = UXR_FRAMING_READING_PAYLOAD;
                        }
                    }
                    else if (0 < uxr_framing_read_transport(framing_io, read_cb, cb_arg, timeout, errcode, 1))
                    {

                    }
                    else
                    {
                        if (UXR_FRAMING_BEGIN_FLAG == octet)
                        {
                            framing_io->state = UXR_FRAMING_READING_SRC_ADDR;
                        }
                        else
                        {
                            exit_cond = true;
                        }
                    }
                    break;
                }
                case UXR_FRAMING_READING_PAYLOAD:
                {
                    while ((framing_io->msg_pos < framing_io->msg_len) && uxr_get_next_octet(framing_io, &octet))
                    {
                        buf[(size_t)framing_io->msg_pos] = octet;
                        framing_io->msg_pos = (uint16_t)(framing_io->msg_pos + 1);
                        uxr_update_crc(&framing_io->cmp_crc, octet);
                    }

                    if (framing_io->msg_pos == framing_io->msg_len)
                    {
                        framing_io->state = UXR_FRAMING_READING_CRC_LSB;
                    }
                    else
                    {
                        if (UXR_FRAMING_BEGIN_FLAG == octet)
                        {
                            framing_io->state = UXR_FRAMING_READING_SRC_ADDR;
                        }
                        else if (0 <
                                uxr_framing_read_transport(framing_io, read_cb, cb_arg, timeout, errcode,
                                (size_t)((framing_io->msg_len - framing_io->msg_pos) + 2)))
                        {

                        }
                        else
                        {
                            exit_cond = true;
                        }
                    }
                    break;
                }
                case UXR_FRAMING_READING_CRC_LSB:
                {
                    if (uxr_get_next_octet(framing_io, &octet))
                    {
                        framing_io->msg_crc = octet;
                        framing_io->state = UXR_FRAMING_READING_CRC_MSB;
                    }
                    else if (0 < uxr_framing_read_transport(framing_io, read_cb, cb_arg, timeout, errcode, 2))
                    {

                    }
                    else
                    {
                        if (UXR_FRAMING_BEGIN_FLAG == octet)
                        {
                            framing_io->state = UXR_FRAMING_READING_SRC_ADDR;
                        }
                        else
                        {
                            exit_cond = true;
                        }
                    }
                    break;
                }
                case UXR_FRAMING_READING_CRC_MSB:
                {
                    if (uxr_get_next_octet(framing_io, &octet))
                    {
                        framing_io->msg_crc = (uint16_t)(framing_io->msg_crc + (octet << 8));
                        framing_io->state = UXR_FRAMING_UNINITIALIZED;
                        if (framing_io->cmp_crc == framing_io->msg_crc)
                        {
                            *remote_addr = framing_io->src_addr;
                            rv = framing_io->msg_len;
                        }
                        exit_cond = true;
                    }
                    else if (0 < uxr_framing_read_transport(framing_io, read_cb, cb_arg, timeout, errcode, 1))
                    {

                    }
                    else
                    {
                        if (UXR_FRAMING_BEGIN_FLAG == octet)
                        {
                            framing_io->state = UXR_FRAMING_READING_SRC_ADDR;
                        }
                        else
                        {
                            exit_cond = true;
                        }
                    }
                    break;
                }
                default:
                    break;
            }
        }
    }

    return rv;
}
bool uxr_read_framed_msg2(
        uxrFramingIO* framing_io,
        uxr_read_cb_new read_cb_new,
        uxr_write_to_dma_buffer write_to_dma_buffer,
        void* cb_arg,
        uint8_t* buf,
        size_t len,
        int pos,
        int length,
        int* timeout,
        uint8_t* errcode)
{
    //记录数据包的起始位置
    int temp_pos = pos;
    //重新定位数据包的起始位置
    uint8_t octet;
    read_cb_new(&octet, 1, pos);
    if(octet == UXR_FRAMING_ESC_FLAG){
        pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
    }
    //将该数据包标记为已收集
    write_to_dma_buffer(pos, UXR_HAD_COLLECTED);
    //越过数据包头
    for(int i = 1;i < 5;++i){
        pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
        read_cb_new(&octet, 1, pos);
        if(octet == UXR_FRAMING_ESC_FLAG){
            pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
        }
    }
    framing_io->msg_pos = 0;
    framing_io->cmp_crc = 0;
    //i用来记录已经读取的数据填充部分的长度
    int i = 0;
    pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
    while (i < length)
    {
        read_cb_new(&octet, 1, pos);
        if(octet == UXR_FRAMING_ESC_FLAG){
            pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
            read_cb_new(&octet, 1, pos);
            if (UXR_FRAMING_BEGIN_FLAG != octet)
            {
                octet ^= UXR_FRAMING_XOR_FLAG;
            }
        }
        buf[(size_t)framing_io->msg_pos] = octet;
        framing_io->msg_pos = (uint16_t)(framing_io->msg_pos + 1);
        //进行CRC校验
        uxr_update_crc(&framing_io->cmp_crc, octet);
        pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
        i++;
    }
    //计算得到CRC的总值
    read_cb_new(&octet, 1, pos);
    while(UXR_FRAMING_ESC_FLAG == octet){
        pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
        read_cb_new(&octet, 1, pos);
    }
    framing_io->msg_crc = octet;
    pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
    read_cb_new(&octet, 1, pos);
    while(UXR_FRAMING_ESC_FLAG == octet){
        pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
        read_cb_new(&octet, 1, pos);
    }
    framing_io->msg_crc = (uint16_t)(((uint8_t)framing_io->msg_crc) | (octet << 8));
    if(temp_pos == dma_head)
    {
        dma_head = (pos + 1) % UART_DMA_BUFFER_SIZE;
        //跟下一个数据包的起始位置对齐
        while(dma_head != dma_tail){
            read_cb_new(&octet, 1, dma_head);
            if(octet == UXR_FRAMING_BEGIN_FLAG || octet == UXR_HEADER_HANDLE_FLAG || octet == UXR_HAD_COLLECTED){
                if(octet == UXR_HAD_COLLECTED){
                    //表明这个数据包已经收集过，需要跳过该数据包
                    HERE:
                    dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                    read_cb_new(&octet, 1, dma_head);
                    if(octet == UXR_FRAMING_ESC_FLAG){
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, dma_head);
                        if (UXR_FRAMING_BEGIN_FLAG != octet)
                        {
                            octet ^= UXR_FRAMING_XOR_FLAG;
                        }
                    }
                    dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                    read_cb_new(&octet, 1, dma_head);
                    if(octet == UXR_FRAMING_ESC_FLAG){
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, dma_head);
                        if (UXR_FRAMING_BEGIN_FLAG != octet)
                        {
                            octet ^= UXR_FRAMING_XOR_FLAG;
                        }
                    }
                    dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                    read_cb_new(&octet, 1, dma_head);
                    if(octet == UXR_FRAMING_ESC_FLAG){
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, dma_head);
                        if (UXR_FRAMING_BEGIN_FLAG != octet)
                        {
                            octet ^= UXR_FRAMING_XOR_FLAG;
                        }
                    }
                    //获得该数据包的长度
                    uint16_t msg_len = octet;
                    dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                    read_cb_new(&octet, 1, dma_head);
                    if(octet == UXR_FRAMING_ESC_FLAG){
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, dma_head);
                        if (UXR_FRAMING_BEGIN_FLAG != octet)
                        {
                            octet ^= UXR_FRAMING_XOR_FLAG;
                        }
                    }
                    msg_len = (uint16_t)(msg_len + (octet << 8));
                    dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                    dma_head = (dma_head + (msg_len + 2)) % UART_DMA_BUFFER_SIZE;
                    //跟下一个数据包的起始位置对齐
                    while(dma_head != dma_tail){
                        read_cb_new(&octet, 1, dma_head);
                        if(octet == UXR_FRAMING_BEGIN_FLAG || octet == UXR_HEADER_HANDLE_FLAG || octet == UXR_HAD_COLLECTED){
                            if(octet == UXR_HAD_COLLECTED){
                                //找到一个未收集过的数据包
                                goto HERE;
                            }
                            break;
                        }else{
                            dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        }
                    }
                }
                break;
            }else{
                dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
            }
        }
    }else{
        if(temp_pos > dma_head && temp_pos - dma_head > 1000){
            //适当抛弃一些数据包
            dma_head = (pos + 1) % UART_DMA_BUFFER_SIZE;
            //跟下一个数据包的起始位置对齐
            while(dma_head != dma_tail){
                read_cb_new(&octet, 1, dma_head);
                if(octet == UXR_FRAMING_BEGIN_FLAG || octet == UXR_HEADER_HANDLE_FLAG || octet == UXR_HAD_COLLECTED){
                    if(octet == UXR_HAD_COLLECTED){
                        //表明这个数据包已经收集过，需要跳过该数据包
                        HERE1:
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, dma_head);
                        if(octet == UXR_FRAMING_ESC_FLAG){
                            dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                            read_cb_new(&octet, 1, dma_head);
                            if (UXR_FRAMING_BEGIN_FLAG != octet)
                            {
                                octet ^= UXR_FRAMING_XOR_FLAG;
                            }
                        }
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, dma_head);
                        if(octet == UXR_FRAMING_ESC_FLAG){
                            dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                            read_cb_new(&octet, 1, dma_head);
                            if (UXR_FRAMING_BEGIN_FLAG != octet)
                            {
                                octet ^= UXR_FRAMING_XOR_FLAG;
                            }
                        }
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, dma_head);
                        if(octet == UXR_FRAMING_ESC_FLAG){
                            dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                            read_cb_new(&octet, 1, dma_head);
                            if (UXR_FRAMING_BEGIN_FLAG != octet)
                            {
                                octet ^= UXR_FRAMING_XOR_FLAG;
                            }
                        }
                        //获得该数据包的长度
                        uint16_t msg_len = octet;
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, dma_head);
                        if(octet == UXR_FRAMING_ESC_FLAG){
                            dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                            read_cb_new(&octet, 1, dma_head);
                            if (UXR_FRAMING_BEGIN_FLAG != octet)
                            {
                                octet ^= UXR_FRAMING_XOR_FLAG;
                            }
                        }
                        msg_len = (uint16_t)(msg_len + (octet << 8));
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        dma_head = (dma_head + (msg_len + 2)) % UART_DMA_BUFFER_SIZE;
                        //跟下一个数据包的起始位置对齐
                        while(dma_head != dma_tail){
                            read_cb_new(&octet, 1, dma_head);
                            if(octet == UXR_FRAMING_BEGIN_FLAG || octet == UXR_HEADER_HANDLE_FLAG || octet == UXR_HAD_COLLECTED){
                                if(octet == UXR_HAD_COLLECTED){
                                    //找到一个未收集过的数据包
                                    goto HERE1;
                                }
                                break;
                            }else{
                                dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                            }
                        }
                    }
                    break;
                }else{
                    dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                }
            }
        }else if(temp_pos < dma_head && (UART_DMA_BUFFER_SIZE + temp_pos) - dma_head > 1000){
            //适当抛弃一些数据包
            dma_head = (pos + 1) % UART_DMA_BUFFER_SIZE;
            //跟下一个数据包的起始位置对齐
            while(dma_head != dma_tail){
                read_cb_new(&octet, 1, dma_head);
                if(octet == UXR_FRAMING_BEGIN_FLAG || octet == UXR_HEADER_HANDLE_FLAG || octet == UXR_HAD_COLLECTED){
                    if(octet == UXR_HAD_COLLECTED){
                        //表明这个数据包已经收集过，需要跳过该数据包
                        HERE2:
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, dma_head);
                        if(octet == UXR_FRAMING_ESC_FLAG){
                            dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                            read_cb_new(&octet, 1, dma_head);
                            if (UXR_FRAMING_BEGIN_FLAG != octet)
                            {
                                octet ^= UXR_FRAMING_XOR_FLAG;
                            }
                        }
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, dma_head);
                        if(octet == UXR_FRAMING_ESC_FLAG){
                            dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                            read_cb_new(&octet, 1, dma_head);
                            if (UXR_FRAMING_BEGIN_FLAG != octet)
                            {
                                octet ^= UXR_FRAMING_XOR_FLAG;
                            }
                        }
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, dma_head);
                        if(octet == UXR_FRAMING_ESC_FLAG){
                            dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                            read_cb_new(&octet, 1, dma_head);
                            if (UXR_FRAMING_BEGIN_FLAG != octet)
                            {
                                octet ^= UXR_FRAMING_XOR_FLAG;
                            }
                        }
                        //获得该数据包的长度
                        uint16_t msg_len = octet;
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, dma_head);
                        if(octet == UXR_FRAMING_ESC_FLAG){
                            dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                            read_cb_new(&octet, 1, dma_head);
                            if (UXR_FRAMING_BEGIN_FLAG != octet)
                            {
                                octet ^= UXR_FRAMING_XOR_FLAG;
                            }
                        }
                        msg_len = (uint16_t)(msg_len + (octet << 8));
                        dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                        dma_head = (dma_head + (msg_len + 2)) % UART_DMA_BUFFER_SIZE;
                        //跟下一个数据包的起始位置对齐
                        while(dma_head != dma_tail){
                            read_cb_new(&octet, 1, dma_head);
                            if(octet == UXR_FRAMING_BEGIN_FLAG || octet == UXR_HEADER_HANDLE_FLAG || octet == UXR_HAD_COLLECTED){
                                if(octet == UXR_HAD_COLLECTED){
                                    //找到一个未收集过的数据包
                                    goto HERE2;
                                }
                                break;
                            }else{
                                dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                            }
                        }
                    }
                    break;
                }else{
                    dma_head = (dma_head + 1) % UART_DMA_BUFFER_SIZE;
                }
            }
        }
    }
    if(framing_io->msg_crc == framing_io->cmp_crc){
        return true;
    }
    // }else{
    //     printf("WRONG CRC !!!%d  %d\n", framing_io->msg_crc, framing_io->cmp_crc);
    // }
    return true;
}
//目的是获得数据包的长度和数据填充部分的前20个字节
//如果该未被收集的数据包未经过处理，获得数据包的长度和数据填充部分的前20个字节
//如果该未被收集的数据包经过处理，直接获得数据包的长度和优先级
//数据包经过处理后会被标记，数据包的长度和优先级会写入数据包头部
size_t uxr_read_framed_msg_new(
        uxrFramingIO* framing_io,
        uxr_read_cb_new read_cb_new,
        void* cb_arg,
        uint8_t* buf,
        size_t len,
        int* priority,
        uint8_t* remote_addr,
        int* timeout,
        uint8_t* errcode)
{
    if(temp_tail != dma_tail){
        // 存在未收集未处理数据包
        uint8_t octet;
        bool exit_cond = false;
        framing_io->state = UXR_FRAMING_UNINITIALIZED;
        //flag用来标记该数据包头部是否被处理过
        int flag = 0;
        framing_io->msg_len = 0;
        //先判断是否已经接收到数据包头的五个字节
        if(!((dma_tail > temp_tail && dma_tail - temp_tail >= 5) || (dma_tail < temp_tail && dma_tail + UART_DMA_BUFFER_SIZE - temp_tail >= 5))){
            return 0;
        }
        while(!exit_cond){
            switch(framing_io->state){
                case UXR_FRAMING_UNINITIALIZED:
                {
                    read_cb_new(&octet, 1, temp_tail);
                    if(octet == UXR_FRAMING_ESC_FLAG){
                        temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, temp_tail);
                        if (UXR_FRAMING_BEGIN_FLAG != octet)
                        {
                            octet ^= UXR_FRAMING_XOR_FLAG;
                        }
                    }
                    if(octet ==  UXR_FRAMING_BEGIN_FLAG){
                        //数据包头部未被处理过
                        //标记数据包头部未被处理过
                        flag = 1;
                        //数据包头部不包含优先级信息，所以将数据包的优先级置成0
                        (*priority) = 0;
                        framing_io->state = UXR_FRAMING_READING_SRC_ADDR;
                        temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                        break;
                    }else if(octet == UXR_HEADER_HANDLE_FLAG){
                        //数据包头部被处理过
                        temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, temp_tail);
                        if(octet == UXR_FRAMING_ESC_FLAG){
                            temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                            read_cb_new(&octet, 1, temp_tail);
                            if (UXR_FRAMING_BEGIN_FLAG != octet)
                            {
                                octet ^= UXR_FRAMING_XOR_FLAG;
                            }
                        }
                        //通过计算，直接得到数据包的优先级
                        (*priority) = octet;
                        temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, temp_tail);
                        if(octet == UXR_FRAMING_ESC_FLAG){
                            temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                            read_cb_new(&octet, 1, temp_tail);
                            if (UXR_FRAMING_BEGIN_FLAG != octet)
                            {
                                octet ^= UXR_FRAMING_XOR_FLAG;
                            }
                        }
                        (*priority) = ((uint16_t)(*priority) | (octet << 8));
                        framing_io->state = UXR_FRAMING_READING_LEN_LSB;
                        temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                        break;
                    }else if(octet == UXR_HAD_COLLECTED){
                        //数据包已经被搜集
                        //将数据包的优先级置成-1，标记该数据包已经被收集过
                        (*priority) = -1;
                        temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, temp_tail);
                        if(octet == UXR_FRAMING_ESC_FLAG){
                            temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                            read_cb_new(&octet, 1, temp_tail);
                            if (UXR_FRAMING_BEGIN_FLAG != octet)
                            {
                                octet ^= UXR_FRAMING_XOR_FLAG;
                            }
                        }
                        temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, temp_tail);
                        if(octet == UXR_FRAMING_ESC_FLAG){
                            temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                            read_cb_new(&octet, 1, temp_tail);
                            if (UXR_FRAMING_BEGIN_FLAG != octet)
                            {
                                octet ^= UXR_FRAMING_XOR_FLAG;
                            }
                        }
                        framing_io->state = UXR_FRAMING_READING_LEN_LSB;
                        temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                        break;
                    }else{
                        printf("WRONG start !!!\n");
                    }
                }
                case UXR_FRAMING_READING_SRC_ADDR:
                {
                    read_cb_new(&octet, 1, temp_tail);
                    if(octet == UXR_FRAMING_ESC_FLAG){
                        temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, temp_tail);
                        if (UXR_FRAMING_BEGIN_FLAG != octet)
                        {
                            octet ^= UXR_FRAMING_XOR_FLAG;
                        }
                    }
                    (*remote_addr) == octet;
                    framing_io->state = UXR_FRAMING_READING_DST_ADDR;
                    temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                    break;
                }
                case UXR_FRAMING_READING_DST_ADDR:
                {
                    read_cb_new(&octet, 1, temp_tail);
                    if(octet == UXR_FRAMING_ESC_FLAG){
                        temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, temp_tail);
                        if (UXR_FRAMING_BEGIN_FLAG != octet)
                        {
                            octet ^= UXR_FRAMING_XOR_FLAG;
                        }
                    }
                    if(octet != framing_io->local_addr){
                        printf("DST_ADDR ERROR !!!%d   %d\n", framing_io->local_addr, octet);
                    }
                    framing_io->state = UXR_FRAMING_READING_LEN_LSB;
                    temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                    break;;
                }
                case UXR_FRAMING_READING_LEN_LSB:
                {
                    read_cb_new(&octet, 1, temp_tail);
                    if(octet == UXR_FRAMING_ESC_FLAG){
                        temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, temp_tail);
                        if (UXR_FRAMING_BEGIN_FLAG != octet)
                        {
                            octet ^= UXR_FRAMING_XOR_FLAG;
                        }
                    }
                    framing_io->msg_len = octet;
                    framing_io->state = UXR_FRAMING_READING_LEN_MSB;
                    temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                    break;
                }
                case UXR_FRAMING_READING_LEN_MSB:
                {
                    read_cb_new(&octet, 1, temp_tail);
                    if(octet == UXR_FRAMING_ESC_FLAG){
                        temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                        read_cb_new(&octet, 1, temp_tail);
                        if (UXR_FRAMING_BEGIN_FLAG != octet)
                        {
                            octet ^= UXR_FRAMING_XOR_FLAG;
                        }
                    }
                    framing_io->msg_len = (uint16_t)(framing_io->msg_len + (octet << 8));
                    if(flag == 1){
                        //如果数据包未被处理过，获得数据填充部分的前20个字节
                        framing_io->state = UXR_FRAMING_READING_PAYLOAD;
                    }else{
                        //否则，跳出循环
                        exit_cond = true;
                    }
                    temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                    break;
                }
                case UXR_FRAMING_READING_PAYLOAD:
                {
                    //判断是否接收到完整数据包
                    if((temp_tail + (framing_io->msg_len + 2)) % UART_DMA_BUFFER_SIZE > dma_tail){
                        return 0;
                    }
                    //获得数据填充部分的前20个字节
                    // read_cb_new(buf, len, temp_tail);
                    int temp_temp_tail = temp_tail;
                    //获得数据填充部分的前20个字节
                    for(int i = 0;i < len;i++){
                        read_cb_new(&octet, 1, temp_temp_tail);
                        if(octet == UXR_FRAMING_ESC_FLAG){
                            temp_temp_tail = (temp_temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                            read_cb_new(&octet, 1, temp_temp_tail);
                            if (UXR_FRAMING_BEGIN_FLAG != octet)
                            {
                                octet ^= UXR_FRAMING_XOR_FLAG;
                            }
                        }
                        buf[i] = octet;
                        temp_temp_tail = (temp_temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                    }
                    exit_cond = true;
                    break;
                }
            }
        }
        temp_tail = (temp_tail + (framing_io->msg_len + 2)) % UART_DMA_BUFFER_SIZE;
        //跟下一个数据包的起始位置对齐
        while(temp_tail != dma_tail){
            read_cb_new(&octet, 1, temp_tail);
            if(octet == UXR_FRAMING_ESC_FLAG){
                temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
                read_cb_new(&octet, 1, temp_tail);
                if (UXR_FRAMING_BEGIN_FLAG != octet)
                {
                    octet ^= UXR_FRAMING_XOR_FLAG;
                }
            }
            if(octet == UXR_FRAMING_BEGIN_FLAG || octet == UXR_HEADER_HANDLE_FLAG || octet == UXR_HAD_COLLECTED){
                break;
            }else{
                temp_tail = (temp_tail + 1) % UART_DMA_BUFFER_SIZE;
            }
        }
        return framing_io->msg_len;
    }
    return 0;
}
void uxr_write_msg_header(uxr_read_cb_new read_cb_new, uxr_write_to_dma_buffer write_to_dma_buffer, int pos, int priority){
    uint8_t octet;
    read_cb_new(&octet, 1, pos);
    if(octet == UXR_FRAMING_ESC_FLAG){
        pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
    }
    write_to_dma_buffer(pos, UXR_HEADER_HANDLE_FLAG);
    pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
    read_cb_new(&octet, 1, pos);
    if(octet == UXR_FRAMING_ESC_FLAG){
        pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
    }
    //将优先级的低字节写入
    write_to_dma_buffer(pos, (uint8_t)(priority & 0xFF));
    pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
    read_cb_new(&octet, 1, pos);
    while(octet == UXR_FRAMING_ESC_FLAG){
        pos = (pos + 1) % UART_DMA_BUFFER_SIZE;
        read_cb_new(&octet, 1, pos);
    }
    //将优先级的高字节写入
    write_to_dma_buffer(pos, (uint8_t)((priority >> 8) & 0xFF));
}