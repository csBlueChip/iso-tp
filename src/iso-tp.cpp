#include "Arduino.h"
#include "iso-tp.h"
#include <can_common.h>

//----------------------------------------------------------------------------------------------------------------------
#define  DBG_ON (0)  // debug control 0=off, 1=on
#if (DGB_ON == 1) 
#	define DBG(...)     Serial.print(__VA_ARGS__)
#	define DGBLN(...)   Serial.println(__VA_ARGS__)
#	define DGBBUF(...)  print_buffer(__VA_ARGS__)
#else
#	define DBG(...)
#	define DGBLN(...)
#	define DGBBUF(...)  print_buffer(__VA_ARGS__)
#endif	

//+=====================================================================================================================
IsoTp::IsoTp(CAN_COMMON* bus)
{
    _bus = bus;
}

//+=====================================================================================================================
void IsoTp::print_buffer(uint32_t id, uint8_t *buffer, uint16_t len)
{
    uint16_t i=0;

    Serial.print(F("Buffer: "));
    Serial.print(id,HEX);
    Serial.print(F(" ["));
    Serial.print(len);
    Serial.print(F("] "));
    for (i=0; i<len; i++) {
        if (buffer[i] < 0x10) {
            Serial.print(F("0"));
        }
        Serial.print(buffer[i],HEX);
        Serial.print(F(" "));
    }
    Serial.println();
}

//+=====================================================================================================================
uint8_t IsoTp::send(Message_t* msg)
{
    uint8_t bs=false;
    uint32_t delta=0;
    uint8_t retval=0;

    msg->tp_state=ISOTP_SEND;

    while(msg->tp_state!=ISOTP_IDLE && msg->tp_state!=ISOTP_ERROR) {
        bs=false;

        DBG(F("ISO-TP State: "));
        DBGLN(msg->tp_state);
        DBG(F("Length      : "));
        DBGLN(msg->len);

        switch (msg->tp_state) {
        case ISOTP_IDLE         :
				break;
			case ISOTP_SEND         :
				if (msg->len<=7) {
					DBGLN(F("Send SF"));
					retval=send_sf(msg);
					msg->tp_state=ISOTP_IDLE;
				} else {
					DBGLN(F("Send FF"));
					if (!(retval=send_ff(msg))) {  // FF complete
						msg->Buffer+=6;
						msg->len-=6;
						msg->tp_state=ISOTP_WAIT_FIRST_FC;
						fc_wait_frames=0;
						wait_fc=millis();
					}
				}
				break;
			case ISOTP_WAIT_FIRST_FC:
				DBGLN(F("Wait first FC"));
				delta=millis()-wait_fc;
				if (delta >= TIMEOUT_FC) {
					DBG(F("FC timeout during receive"));
					DBG(F(" wait_fc="));
					DBG(wait_fc);
					DBG(F(" delta="));
					DBGLN(delta);
					msg->tp_state = ISOTP_IDLE;
					retval=1;
				}
				break;
			case ISOTP_WAIT_FC      :
				DBGLN(F("Wait FC"));
				break;
			case ISOTP_SEND_CF      :
				DBGLN(F("Send CF"));
				while (msg->len>7 & !bs) {
					fc_delay(msg->min_sep_time);
					if (!(retval=send_cf(msg))) {
						DBG(F("Send Seq "));
						DBGLN(msg->seq_id);
						if (msg->blocksize > 0) {
							DBG(F("Blocksize trigger "));
							DBG(msg->seq_id % msg->blocksize);
							if (!(msg->seq_id % msg->blocksize)) {
								bs=true;
								msg->tp_state=ISOTP_WAIT_FC;
								DBGLN(F(" yes"));
							} else {
								DBGLN(F(" no"));
							}
						}
						msg->seq_id++;
						msg->seq_id %= 16;
						msg->Buffer+=7;
						msg->len-=7;
						DBG(F("Length      : "));
						DBGLN(msg->len);
					}
				}
				if (!bs) {
					fc_delay(msg->min_sep_time);
					DBG(F("Send last Seq "));
					DBGLN(msg->seq_id);
					retval=send_cf(msg);
					msg->tp_state=ISOTP_IDLE;
				}
				break;
			default                 :
				break;
        }


        if ((msg->tp_state==ISOTP_WAIT_FIRST_FC) || (msg->tp_state==ISOTP_WAIT_FC)) {
            if (can_receive()) {
                DBGLN(F("Send branch:"));
                if (rcvFrame.id == msg->rx_id) {
                    retval=rcv_fc(msg);
                    memset(rcvFrame.data.bytes,0,sizeof(rcvFrame.data.value));
                    DBGLN(F("rxId OK!"));
                }
            }
        }
    }

    return retval;
}

//+=====================================================================================================================
uint8_t IsoTp::receive(Message_t* msg)
{
    uint8_t n_pci_type=0;
    uint32_t delta=0;

    wait_session=millis();
    DBGLN(F("Start receive..."));
    msg->tp_state=ISOTP_IDLE;

    while(msg->tp_state!=ISOTP_FINISHED && msg->tp_state!=ISOTP_ERROR) {
        delta=millis()-wait_session;
        if (delta >= TIMEOUT_SESSION){
            DBG(F("ISO-TP Session timeout wait_session="));
            DBG(wait_session);
            DBG(F(" delta="));
            DBGLN(delta);
            return 1;
        }

        if (can_receive()) {
            if (rcvFrame.id == msg->rx_id) {
                DBGLN(F("rxId OK!"));
                n_pci_type = rcvFrame.data.byte[0] & 0xF0;

                switch (n_pci_type) {
					case N_PCI_FC:
						DBGLN(F("FC"));
						/* tx path: fc frame */
						rcv_fc(msg);
						break;
	
					case N_PCI_SF:
						DBGLN(F("SF"));
						/* rx path: single frame */
						rcv_sf(msg);
	//		      msg->tp_state=ISOTP_FINISHED;
						break;
	
					case N_PCI_FF:
						DBGLN(F("FF"));
						/* rx path: first frame */
						rcv_ff(msg);
	//		      msg->tp_state=ISOTP_WAIT_DATA;
						break;
						break;
	
					case N_PCI_CF:
						DBGLN(F("CF"));
						/* rx path: consecutive frame */
						rcv_cf(msg);
						break;
                }
                memset(rcvFrame.data.bytes,0,sizeof(rcvFrame.data.value));
            }
        }
    }
    DBGLN(F("ISO-TP message received:"));
    DBGBUF(msg->rx_id, msg->Buffer, msg->len);

    return 0;
}

//+=====================================================================================================================
uint8_t IsoTp::can_send(uint32_t id, uint8_t len, uint8_t *data)
{
    DBGLN(F("Send CAN RAW Data:"));
    DBGBUF(id, data, len);
    CAN_FRAME frame;
    frame.id = id;
    frame.rtr = 0;
    frame.extended = false;
    frame.length = len;
    memset(&frame.data.value,FILL_CHAR,8);
    for (int x = 0; x < len; x++)
        frame.data.byte[x] = data[x];
    return _bus->sendFrame(frame);
}

//+=====================================================================================================================
uint8_t IsoTp::can_receive(void)
{

    if (_bus->rx_avail()) {
        memset(&rcvFrame.data.value,FILL_CHAR,8);
        _bus->read(rcvFrame);
        DBGLN(F("Received CAN RAW Data:"));
        DBGBUF(rcvFrame.id, rcvFrame.data.bytes, rcvFrame.length);
        return true;
    } else {
        return false;
    }
}

//+=====================================================================================================================
uint8_t IsoTp::send_fc(struct Message_t *msg)
{
    uint8_t TxBuf[8]= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    // FC message high nibble = 0x3 , low nibble = FC Status
    TxBuf[0]=(N_PCI_FC | msg->fc_status);
    TxBuf[1]=msg->blocksize;
    /* fix wrong separation time values according spec */
    if ((msg->min_sep_time > 0x7F) && ((msg->min_sep_time < 0xF1)
                                       || (msg->min_sep_time > 0xF9)))
    {
        msg->min_sep_time = 0x7F;
    }
    TxBuf[2]=msg->min_sep_time;
    return can_send(msg->tx_id,8,TxBuf);
}

//+=====================================================================================================================
uint8_t IsoTp::send_sf(struct Message_t *msg) //Send SF Message
{
    uint8_t TxBuf[8]= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    // SF message high nibble = 0x0 , low nibble = Length
    TxBuf[0]=(N_PCI_SF | msg->len);
    memcpy(TxBuf+1,msg->Buffer,msg->len);
//  return can_send(msg->tx_id,msg->len+1,TxBuf);// Add PCI length
    return can_send(msg->tx_id,8,TxBuf);// Always send full frame
}

//+=====================================================================================================================
uint8_t IsoTp::send_ff(struct Message_t *msg) // Send FF
{
    uint8_t TxBuf[8]= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    msg->seq_id=1;

    TxBuf[0]=(N_PCI_FF | ((msg->len&0x0F00) >> 8));
    TxBuf[1]=(msg->len&0x00FF);
    memcpy(TxBuf+2,msg->Buffer,6);             // Skip 2 Bytes PCI
    return can_send(msg->tx_id,8,TxBuf);       // First Frame has full length
}

//+=====================================================================================================================
uint8_t IsoTp::send_cf(struct Message_t *msg) // Send CF Message
{
    uint8_t TxBuf[8]= {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint16_t len=7;

    TxBuf[0]=(N_PCI_CF | (msg->seq_id & 0x0F));
	len = (msg->len > 7) ? 7 : msg->len ;

    memcpy(TxBuf+1,msg->Buffer,len);         // Skip 1 Byte PCI
    //return can_send(msg->tx_id,len+1,TxBuf); // Last frame is probably shorter
    // than 8 -> Signals last CF Frame
    return can_send(msg->tx_id,8,TxBuf);     // Last frame is probably shorter
    // than 8, pad out to 8 bytes
}

//+=====================================================================================================================
void IsoTp::fc_delay(uint8_t sep_time)
{
    if (sep_time < 0x80)  delay(sep_time) ;
    else                  delayMicroseconds((sep_time - 0xF0) * 100) ;
}

//+=====================================================================================================================
uint8_t IsoTp::rcv_sf(struct Message_t* msg)
{
    /* get the SF_DL from the N_PCI byte */
    msg->len = rcvFrame.data.byte[0] & 0x0F;
    /* copy the received data bytes */
    memcpy(msg->Buffer, &rcvFrame.data.byte[1],msg->len); // Skip PCI, SF uses len bytes
    msg->tp_state=ISOTP_FINISHED;

    return 0;
}

//+=====================================================================================================================
uint8_t IsoTp::rcv_ff(struct Message_t* msg)
{
    msg->seq_id=1;

    /* get the FF_DL */
    msg->len = (rcvFrame.data.byte[0] & 0x0F) << 8;
    msg->len += rcvFrame.data.byte[1];
    rest=msg->len;

    /* copy the first received data bytes */
    memcpy(msg->Buffer, &rcvFrame.data.byte[2],6); // Skip 2 bytes PCI, FF must have 6 bytes!
    rest-=6; // Restlength

    msg->tp_state = ISOTP_WAIT_DATA;

    DBG(F("First frame received with message length: "));
    DBGLN(rest);
    DBGLN(F("Send flow controll."));
    DBG(F("ISO-TP state: "));
    DBGLN(msg->tp_state);

    /* send our first FC frame with Target Address*/
    struct Message_t fc;
    fc.tx_id=msg->tx_id;
    fc.fc_status=ISOTP_FC_CTS;
    fc.blocksize=0;
    fc.min_sep_time=0;
    return send_fc(&fc);
}

//+=====================================================================================================================
uint8_t IsoTp::rcv_cf(struct Message_t* msg)
{
    //Handle Timeout
    //If no Frame within 250ms change State to ISOTP_IDLE
    uint32_t delta=millis()-wait_cf;

    if ((delta >= TIMEOUT_FC) && msg->seq_id>1) {
        DBGLN(F("CF frame timeout during receive wait_cf="));
        DBG(wait_cf);
        DBG(F(" delta="));
        DBGLN(delta);
        msg->tp_state = ISOTP_IDLE;
        return 1;
    }
    wait_cf=millis();

    DBG(F("ISO-TP state: "));
    DBGLN(msg->tp_state);
    DBG(F("CF received with message rest length: "));
    DBGLN(rest);

    if (msg->tp_state != ISOTP_WAIT_DATA)  return 0 ;

    if ((rcvFrame.data.byte[0] & 0x0F) != (msg->seq_id & 0x0F)) {
        DBG(F("Got sequence ID: "));
        DBG(rcvFrame.data.byte[0] & 0x0F);
        DBG(F(" Expected: "));
        DBGLN(msg->seq_id & 0x0F);
        msg->tp_state = ISOTP_IDLE;
        msg->seq_id = 1;
        return 1;
    }

    if (rest<=7) {  // Last Frame
        memcpy(msg->Buffer+6+7*(msg->seq_id-1), &rcvFrame.data.byte[1], rest);// 6 Bytes in FF +7
        msg->tp_state=ISOTP_FINISHED;                           // per CF skip PCI
        DBG(F("Last CF received with seq. ID: "));
        DBGLN(msg->seq_id);
    } else {
        DBG(F("CF received with seq. ID: "));
        DBGLN(msg->seq_id);
        memcpy(msg->Buffer+6+7*(msg->seq_id-1), &rcvFrame.data.byte[1],7); // 6 Bytes in FF +7
        // per CF
        rest-=7; // Got another 7 Bytes of Data;
    }

    msg->seq_id++;

    return 0;
}

//+=====================================================================================================================
uint8_t IsoTp::rcv_fc(struct Message_t* msg)
{
    uint8_t retval=0;

    if (msg->tp_state != ISOTP_WAIT_FC && msg->tp_state != ISOTP_WAIT_FIRST_FC)  return 0 ;

    /* get communication parameters only from the first FC frame */
    if (msg->tp_state == ISOTP_WAIT_FIRST_FC) {
        msg->blocksize = rcvFrame.data.byte[1];
        msg->min_sep_time = rcvFrame.data.byte[2];

        /* fix wrong separation time values according spec */
        if ((msg->min_sep_time > 0x7F) && ((msg->min_sep_time < 0xF1)
                                           || (msg->min_sep_time > 0xF9)))
        {
            msg->min_sep_time = 0x7F;
        }
    }

    DBG(F("FC frame: FS "));
    DBG(rcvFrame.data.byte[0]&0x0F);
    DBG(F(", Blocksize "));
    DBG(msg->blocksize);
    DBG(F(", Min. separation Time "));
    DBGLN(msg->min_sep_time);

    switch (rcvFrame.data.byte[0] & 0x0F) {
		case ISOTP_FC_CTS:
			msg->tp_state = ISOTP_SEND_CF;
			break;
		case ISOTP_FC_WT:
			fc_wait_frames++;
			if (fc_wait_frames >= MAX_FCWAIT_FRAME) {
				DBGLN(F("FC wait frames exceeded."));
				fc_wait_frames=0;
				msg->tp_state = ISOTP_IDLE;
				retval=1;
			}
			DBGLN(F("Start waiting for next FC"));
			break;
		case ISOTP_FC_OVFLW:
			DBGLN(F("Overflow in receiver side"));
		default:
			msg->tp_state = ISOTP_IDLE;
			retval=1;
	}
    return retval;
}
