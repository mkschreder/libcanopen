/*
* Copyright 2017-2019 Martin K. Schröder
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <errno.h>
#include <string.h>

#include <libfirmware/thread.h>
#include <libfirmware/atomic.h>
#include <libfirmware/math.h>

#include "canopen.h"

#include <stdio.h>
#include <limits.h>

#define CANOPEN_CAN_TX_TIMEOUT 100
#define CANOPEN_CAN_RX_TIMEOUT 1
#ifdef __linux__
// on linux things like valgrind can cause problems if we timeout too quickly
#define CANOPEN_LSS_FASTSCAN_TIMEOUT_MS 100
#else
#define CANOPEN_LSS_FASTSCAN_TIMEOUT_MS 10
#endif
#define CANOPEN_LSS_TIMEOUT_MS 50
#define CANOPEN_SDO_TIMEOUT_MS 100

#if defined(DEBUG) && defined(__linux__)
#define canopen_debug(...) do { printf("CANOPEN: " __VA_ARGS__); fflush(stdout); } while(0)
#else
#define canopen_debug(...) do {} while(0)
#endif

#define canopen_msg_get_cob(msg) ((msg)->id & (uint32_t)~(uint32_t)0x7f)

static uint16_t co16_to_u16(const uint8_t *data){
	return (uint16_t)(
		((uint16_t)data[0] << 0) |
		((uint16_t)data[1] << 8));
}

static uint32_t co24_to_u24(const uint8_t *data){
	return (uint32_t)(
		((uint32_t)data[0] << 0) |
		((uint32_t)data[1] << 8) |
		((uint32_t)data[2] << 16)
	);
}

static uint32_t co32_to_u32(const uint8_t *data){
	return (uint32_t)(
		((uint32_t)data[0] << 0) |
		((uint32_t)data[1] << 8) |
		((uint32_t)data[2] << 16) |
		((uint32_t)data[3] << 24)
	);
}

static void u16_to_co16(uint16_t val, uint8_t *data){
	data[0] = (uint8_t)(val);
	data[1] = (uint8_t)(val >> 8);
}

static void u32_to_co32(uint32_t val, uint8_t *data){
	data[0] = (uint8_t)(val);
	data[1] = (uint8_t)(val >> 8);
	data[2] = (uint8_t)(val >> 16);
	data[3] = (uint8_t)(val >> 24);
}

int canopen_lss_reset(struct canopen *self){
	struct can_message msg;
	can_message_init(&msg);
	msg.id = CANOPEN_COB_LSS | CANOPEN_LSS_TX;
	msg.len = 8;
	msg.data[0] = CANOPEN_LSS_CMD_RESET;

	// this message is not confirmed
	if(can_send(self->port, &msg, CANOPEN_CAN_TX_TIMEOUT) < 0) return -1;
	return 0;
}

int canopen_lss_mode(struct canopen *self, uint8_t mode){
	struct can_message msg;
	can_message_init(&msg);
	msg.id = CANOPEN_COB_LSS | CANOPEN_LSS_TX;
	msg.len = 8;
	msg.data[0] = CANOPEN_LSS_CMD_SWITCH_MODE;
	msg.data[1] = mode;

	if(can_send(self->port, &msg, CANOPEN_CAN_TX_TIMEOUT) < 0) return -1;
	return 0;
}

static int _lss_scan_request(struct canopen *self){
	// this way we can home in on one single node on the network
	canopen_debug("LSS master: comparing part %d: %08x\n", self->lss.part, self->lss.id & (0xffffffff << self->lss.bit));

	// setup timer and enter wait state
	self->lss.confirmed = false;
	self->lss.timeout = micros() + CANOPEN_LSS_FASTSCAN_TIMEOUT_MS * 1000U;
	self->lss.state = CANOPEN_LSS_STATE_SCAN_WAIT_CONFIRM;

	// send out lss message
	struct can_message msg;
	can_message_init(&msg);
	msg.id = CANOPEN_COB_LSS | CANOPEN_LSS_TX;
	msg.len = 8;
	msg.data[0] = CANOPEN_LSS_CMD_FASTSCAN;
	u32_to_co32(self->lss.id, &msg.data[1]);
	msg.data[5] = (uint8_t)self->lss.bit;
	msg.data[6] = self->lss.part;

	// if last bit then we send next part
	if(self->lss.bit == 0 && self->lss.part != 3){
		msg.data[7] = (uint8_t)(self->lss.part+1);
	} else {
		msg.data[7] = self->lss.part;
	}

	if(can_send(self->port, &msg, CANOPEN_CAN_TX_TIMEOUT) < 0){
		// TODO: abort scan and mark lss request completed
		COVERAGE_DUMMY();
	}

	return 0;
}

static void _canopen_lss_complete_request(struct canopen *self){
	// mark request as completed and reset the lss state machine
	self->lss.req.running = false;
	self->lss.state = CANOPEN_LSS_STATE_OFF;
	thread_sem_give(&self->lss.req.done);
}

static void _process_lss_timeouts(struct canopen *self){
	switch(self->lss.state){
		case CANOPEN_LSS_STATE_SCAN_WAIT_CONFIRM: {
			// we must always wait for timeout because multiple nodes may respond to the same message. We want to wait until all nodes have responded.
			if(time_after(micros(), self->lss.timeout)) {
				if(self->lss.bit >= 0){
					if(self->lss.confirmed){
						canopen_debug("LSS master: %d bits confirmed\n", 32 - self->lss.bit);
					} else {
						// if bit has not been confirmed then we toggle it
						canopen_debug("LSS master: %d bits not confirmed\n", 32 - self->lss.bit);
						self->lss.id |= (uint32_t)(1 << (self->lss.bit));
					}

					// we currently do not retry for 0 bits so we store the part here. Not sure if this is entirely according to spec.
					//self->lss.req.serial[self->lss.part] = self->lss.id;
				}

				// upon timeout we either set next part to scan or exit lss scanning and mark the request as completed
				if(self->lss.bit > 0){
					self->lss.bit = (self->lss.bit - self->lss.bits_per_test) % 31;
					_lss_scan_request(self);
				} else if(self->lss.bit == 0 && self->lss.part < 3){
					canopen_debug("Current part checked %08x\n", self->lss.id);
					self->lss.req.serial[self->lss.part] = self->lss.id;
					self->lss.part++;
					self->lss.bit = (self->lss.bit - self->lss.bits_per_test) % 31;
					if(self->lss.bit < 0) self->lss.bit += 32;
					self->lss.id = self->lss.req.serial[self->lss.part];
					canopen_debug("Next part to check %08x, bit %d\n", self->lss.id, self->lss.bit);
					_lss_scan_request(self);
				} else {
					self->lss.req.serial[self->lss.part] = self->lss.id;
					if(
						self->lss.req.serial[0] == UINT_MAX && 
						self->lss.req.serial[1] == UINT_MAX && 
						self->lss.req.serial[2] == UINT_MAX && 
						self->lss.req.serial[3] == UINT_MAX){
						self->lss.req.result = -1;
					} else {
						self->lss.req.result = 0;
					}
					canopen_debug("LSS request completed\n");
					_canopen_lss_complete_request(self);
				}
			}
		} break;
		case CANOPEN_LSS_STATE_ENABLE_WAIT_CONFIRM: {
			// wait for timeout regardless of whether we get a reply or not. Since LSS works this way.
			if(time_after(micros(), self->lss.timeout)) {
				// if no node confirmed the request then we bail out with an error
				if(!self->lss.confirmed){
					self->lss.req.result = -ETIMEDOUT;
					_canopen_lss_complete_request(self);
				} else if(self->lss.part < 4){
					self->lss.part++;
					_lss_scan_request(self);
				} else {
					// request is confirmed and this is last part so complete with success
					// send command to enable lss on currently selected node
					// this presupposes that serial numbers really are unique (otherwise we would enable it on two nodes. bad)
					//canopen_lss_mode(self, 1); // in lss this never gets confirmed
					self->lss.req.result = 0;
					_canopen_lss_complete_request(self);
				}
			}
		} break;
		case CANOPEN_LSS_STATE_SET_ID_WAIT_CONFIRM: {
			// id 
			if(time_after(micros(), self->lss.timeout)){
				// the request timed out
				self->lss.req.result = -ETIMEDOUT;
				_canopen_lss_complete_request(self);
			}
		} break;
		default: break;
	}
}

static void _handle_message_lss(struct canopen *self, struct can_message *rx_msg){
	switch(self->lss.state){
		case CANOPEN_LSS_STATE_OFF: {
		} break;
		case CANOPEN_LSS_STATE_SCAN_WAIT_CONFIRM: {
			canopen_debug("LSS master: got scan confirmation for %d bits\n", 32 - self->lss.bit);
			canopen_debug("msg cob: %04x, data[0] = %02x\n", rx_msg->id, rx_msg->data[0]);
			// if any node replies with an ok then we note that we have an ok. We may get several replies so we must wait until all nodes reply.
			if(rx_msg && rx_msg->id == (CANOPEN_COB_LSS | CANOPEN_LSS_RX) && rx_msg->data[0] == CANOPEN_LSS_CMD_FASTSCAN){
				// bit confirmed, but do nothing. We wait for timeout anyway.
				self->lss.confirmed = true;
			}
			// rest is handled in timeout handler
		} break;
		case CANOPEN_LSS_STATE_ENABLE_WAIT_CONFIRM: {
			canopen_debug("LSS master: got enable confirmation\n");
			// since several nodes may reply to the same request, we must wait until all of them have had chance to reply
			if(rx_msg && rx_msg->id == (CANOPEN_COB_LSS | CANOPEN_LSS_RX) && rx_msg->data[0] == CANOPEN_LSS_CMD_FASTSCAN) {
				// confirmation received
				self->lss.confirmed = true;
			}
		} break;
		case CANOPEN_LSS_STATE_SET_ID_WAIT_CONFIRM: {
			canopen_debug("LSS master: set id confirmation\n");
			if(rx_msg && (rx_msg->data[1] != 0 || rx_msg->data[2] != 0)){
				// got an error response
				self->lss.req.result = -1;
				_canopen_lss_complete_request(self);
			} else if(rx_msg && rx_msg->id == (CANOPEN_COB_LSS | CANOPEN_LSS_RX) && rx_msg->data[0] == CANOPEN_LSS_CMD_SET_ID) {
				// confirmed
				_canopen_lss_complete_request(self);
			}
		} break;
		default: break;
	}
}

#define PDO_SEND_SYNC 1
#define PDO_SEND_ASYNC 2

static void _send_pending_pdos(struct canopen *self, uint32_t pdo_types){
	for(uint16_t c = 0; c < CANOPEN_DEFAULT_TXPDO_COUNT; c++){
		struct canopen_pdo_entry *pdo = &self->profile.txpdo[c];
		if(!pdo->map_entries) continue;
		struct can_message tmsg;
		can_message_init(&tmsg);
		tmsg.id = pdo->cob_id;
		tmsg.len = 8;
		uint32_t pos = 0;
		for(uint32_t m_id = 0; m_id < pdo->map_entries; m_id++){
			uint32_t m = pdo->map[m_id];
			uint32_t idx = (uint32_t)(m >> 8);
			uint32_t value = 0;
			if(vardir_get_u32(self->vardir, idx, &value) < 0){
				// TODO: log failure
			}
			switch(m & 0xff){
				case CANOPEN_PDO_SIZE_32: {
					if(pos + 4 > sizeof(tmsg.data)) break;
					u32_to_co32(value, &tmsg.data[pos]);
					pos += 4;
				} break;
				case CANOPEN_PDO_SIZE_16: {
					if(pos + 2 > sizeof(tmsg.data)) break;
					u16_to_co16((uint16_t)value, &tmsg.data[pos]);
					pos += 2;
				} break;
				case CANOPEN_PDO_SIZE_8: {
					if(pos + 1 > sizeof(tmsg.data)) break;
					tmsg.data[pos] = (uint8_t)value;
					pos += 1;
				} break;
			}
		}
		// the PDO CAN be shorter than 8 bytes.
		tmsg.len = (uint8_t)pos;
		bool is_same = memcmp(pdo->last, tmsg.data, sizeof(pdo->last)) == 0;
		memcpy(pdo->last, tmsg.data, sizeof(pdo->last));

		if(pdo->sync_cycles > 0 && pdo->type >= CANOPEN_PDO_TYPE_CYCLIC(1) && pdo->type <= CANOPEN_PDO_TYPE_CYCLIC(240)){
			pdo->sync_cycles--;
		}

		if(
			((pdo_types & PDO_SEND_SYNC) && (
				(pdo->transmit && pdo->type == CANOPEN_PDO_TYPE_ACYCLIC) || // if the pdo is marked for transmission and the type is acyclic
				(pdo->sync_cycles == 0 && pdo->type >= CANOPEN_PDO_TYPE_CYCLIC(1) && pdo->type <= CANOPEN_PDO_TYPE_CYCLIC(240)) // if sync cycles timer has expired and this is a synchronous pdo
			)) ||
			((pdo_types & PDO_SEND_ASYNC) && (
				(!is_same && pdo->type >= CANOPEN_PDO_TYPE_ASYNC) // if the data has changed
			))
		){
			if(can_send(self->port, &tmsg, CANOPEN_CAN_TX_TIMEOUT) < 0){
				// TODO: log failure
			}
			// pdo type is number jj
			if(pdo->type >= CANOPEN_PDO_TYPE_CYCLIC(1) && pdo->type <= CANOPEN_PDO_TYPE_CYCLIC(240)){
				pdo->sync_cycles = pdo->type;
			}
		}
	}
}

static void _handle_message(struct can_listener *listener, struct can_message *msg){
	struct canopen *self = container_of(listener, struct canopen, listener);

	uint32_t msg_id = msg->id & 0x7ff;
	uint32_t cob = msg_id & (uint32_t)~(uint32_t)0x7f;
	uint32_t node_id = msg_id & 0x7f;

	switch(cob){
		case CANOPEN_COB_NMT: {

		} break;
		case CANOPEN_COB_SYNC_OR_EMCY: {
			if(msg->len == 0) { // sync
				// send synchronous pdos
				_send_pending_pdos(self, PDO_SEND_SYNC);

				atomic_inc(&self->cnt.sync_in);
			}
			// otherwise it is an emergency message. Call emergency listeners.
			struct canopen_listener *entry;
			list_for_each_entry(entry, &self->emcy_listeners, list){
				if(entry->callback) entry->callback(entry, (uint8_t)node_id, msg);
			}
		} break;
		case CANOPEN_COB_TIME: {

		} break;
		case CANOPEN_COB_TXPDO_0:
		case CANOPEN_COB_TXPDO_1:
		case CANOPEN_COB_TXPDO_2:
		case CANOPEN_COB_TXPDO_3:
		case CANOPEN_COB_RXPDO_0:
		case CANOPEN_COB_RXPDO_1:
		case CANOPEN_COB_RXPDO_2:
		case CANOPEN_COB_RXPDO_3: {
			// parse and store the received pdo
			struct canopen_pdo_entry *pdo = NULL;
			for(uint16_t c = 0; c < CANOPEN_DEFAULT_RXPDO_COUNT; c++){
				pdo = &self->profile.rxpdo[c];
				if(!(pdo->cob_id & CANOPEN_PDO_DISABLED) && pdo->cob_id == msg_id){
					break;
				}
				pdo = NULL;
			}
			if(!pdo) break;
			size_t pos = 0;
			for(uint8_t c = 0; c < pdo->map_entries; c++){
				if(!pdo->map[c]) break;
				uint32_t m = pdo->map[c];
				uint32_t idx = (uint32_t)(m >> 8);
				switch(m & 0xff){
					case CANOPEN_PDO_SIZE_32: {
						if(vardir_set_int(self->vardir, idx, co32_to_u32(&msg->data[pos])) < 0){
							//failed = true;
						}
						pos += 4;
					} break;
					case CANOPEN_PDO_SIZE_16: {
						if(vardir_set_int(self->vardir, idx, co16_to_u16(&msg->data[pos])) < 0){
							//failed = true;
						}
						pos += 2;
					} break;
					case CANOPEN_PDO_SIZE_8: {
						if(vardir_set_int(self->vardir, idx, msg->data[pos]) < 0){
							//failed = true;
						}
						pos += 1;
					} break;
				}
			}
		} break;
		case CANOPEN_COB_TXSDO: {
			// only process these if we have an outstanding request
			if(!self->sdo.req.running) break;
			uint8_t cmd = msg->data[0];
			canopen_debug("SDO response received\n");
			switch(cmd){
				// received data from a read request
				case CANOPEN_SDO_CMD_READ1:
				case CANOPEN_SDO_CMD_READ2:
				case CANOPEN_SDO_CMD_READ3:
				case CANOPEN_SDO_CMD_READ4: {
					uint32_t dic = (uint32_t)(((uint32_t)co16_to_u16(&msg->data[1]) << 8) | msg->data[3]);
					uint8_t len = 0;
					switch(cmd){
						case CANOPEN_SDO_CMD_READ1: len = 1; break;
						case CANOPEN_SDO_CMD_READ2: len = 2; break;
						case CANOPEN_SDO_CMD_READ3: len = 3; break;
						case CANOPEN_SDO_CMD_READ4: len = 4; break;
					}
					// copy the data to the output buffer
					if(self->sdo.req.cmd == CANOPEN_SDO_CMD_READ && dic == self->sdo.req.id){
						memcpy(self->sdo.req.output, &msg->data[4], (self->sdo.req.output_len < len)?self->sdo.req.output_len:len);
						self->sdo.req.len = len;
						self->sdo.req.running = false;
						thread_sem_give(&self->sdo.req.done);
					}
				} break;
				case CANOPEN_SDO_CMD_ABORT: {
					int32_t code = (int32_t)co32_to_u32(&msg->data[4]);
					self->sdo.req.running = false;
					self->sdo.req.result = -code;
					thread_sem_give(&self->sdo.req.done);
				} break;
				// response from node to which we have written
				case CANOPEN_SDO_CMD_WRITE: {
					uint32_t dic = (uint32_t)(((uint32_t)co16_to_u16(&msg->data[1]) << 8) | msg->data[3]);
					// make sure that idx and sub are the same as in the currently pending request
					if(self->sdo.req.id == dic){
						self->sdo.req.result = 0;
					} else {
						self->sdo.req.result = -1;
					}
					self->sdo.req.running = false;
					thread_sem_give(&self->sdo.req.done);
				} break;
			}
		} break;
		case CANOPEN_COB_RXSDO: {
			if(node_id != self->address) break;
			canopen_debug("SDO request received\n");
			// master to slave
			uint8_t cmd = msg->data[0];
			switch(cmd){
				case CANOPEN_SDO_CMD_READ1:
				case CANOPEN_SDO_CMD_READ2:
				case CANOPEN_SDO_CMD_READ3:
				case CANOPEN_SDO_CMD_READ4: {
					uint32_t id = (uint32_t)(((uint32_t)co16_to_u16(&msg->data[1]) << 8) | msg->data[3]);
					uint32_t val = 0;
					struct can_message tmsg;
					can_message_init(&tmsg);
					tmsg.id = (uint32_t)(CANOPEN_COB_TXSDO | self->address);
					tmsg.len = 8;
					tmsg.data[0] = CANOPEN_SDO_CMD_ABORT;
					tmsg.data[1] = msg->data[1];
					tmsg.data[2] = msg->data[2];
					tmsg.data[3] = msg->data[3];

					struct vardir_entry *ent = vardir_find_entry_by_id(self->vardir, id);
					if(ent){
						vardir_entry_get_u32(ent, &val);
						switch(ent->type & VARDIR_VALUE_TYPE_MASK){
							case VAR_INT32:
							case VAR_UINT32:
								tmsg.data[0] = CANOPEN_SDO_CMD_READ4;
								u32_to_co32(val, &tmsg.data[4]);
								break;
							case VAR_INT16:
							case VAR_UINT16:
								tmsg.data[0] = CANOPEN_SDO_CMD_READ2;
								u16_to_co16((uint16_t)val, &tmsg.data[4]);
							break;
							case VAR_INT8:
							case VAR_UINT8:
								tmsg.data[0] = CANOPEN_SDO_CMD_READ1;
								tmsg.data[4] = (uint8_t)(val >> 0);
							break;
							default:
								u32_to_co32(CANOPEN_SDO_ERR_LEN_INVAL, &tmsg.data[4]);
							break;
						}
					} else {
						u32_to_co32(CANOPEN_SDO_ERR_NO_EXIST, &tmsg.data[4]);
					}
					can_send(self->port, &tmsg, CANOPEN_CAN_TX_TIMEOUT);
				} break;
				case CANOPEN_SDO_CMD_WRITE1:
				case CANOPEN_SDO_CMD_WRITE2:
				case CANOPEN_SDO_CMD_WRITE3:
				case CANOPEN_SDO_CMD_WRITE4: {
					// slave to master
					uint16_t sdo = co16_to_u16(&msg->data[1]);
					uint32_t idx = (uint32_t)(((uint32_t)sdo << 8) | msg->data[3]);
					uint32_t val = 0;
					switch(cmd){
						case CANOPEN_SDO_CMD_WRITE4:
							val = co32_to_u32(&msg->data[4]);
							break;
						case CANOPEN_SDO_CMD_WRITE3:
							val = co24_to_u24(&msg->data[4]);
							break;
						case CANOPEN_SDO_CMD_WRITE2:
							val = co16_to_u16(&msg->data[4]);
							break;
						case CANOPEN_SDO_CMD_WRITE1:
							val = ((uint32_t)msg->data[4]);
							break;
					}
					struct can_message tmsg;
					canopen_debug("SDO write %08x %08x\n", idx, val);
					if(vardir_set_int(self->vardir, idx, val) == 0){
						can_message_init(&tmsg);
						tmsg.id = (uint32_t)(CANOPEN_COB_TXSDO | self->address);
						tmsg.len = 8;
						tmsg.data[0] = CANOPEN_SDO_CMD_WRITE;
						tmsg.data[1] = msg->data[1];
						tmsg.data[2] = msg->data[2];
						tmsg.data[3] = msg->data[3];
					} else {
						can_message_init(&tmsg);
						tmsg.id = (uint32_t)(CANOPEN_COB_TXSDO | self->address);
						tmsg.len = 8;
						tmsg.data[0] = CANOPEN_SDO_CMD_ABORT;
						u32_to_co32(CANOPEN_SDO_ERR_NO_EXIST, &tmsg.data[4]);
					}
					can_send(self->port, &tmsg, CANOPEN_CAN_TX_TIMEOUT);
				} break;

			}
		} break;
		case CANOPEN_COB_LSS: {
			if(msg->len != 8) break;
			// slave mode, respond to lss messages
			if(node_id == CANOPEN_LSS_TX){
				uint8_t cmd = msg->data[0];
				switch(cmd){
					case CANOPEN_LSS_CMD_RESET: {
						canopen_debug("LSS: resetting lss\n");
						self->lss.part = 0;
						self->lss.unlocked = false;
						self->lss.enabled = false;
						self->lss.mute = false;
						self->lss.mute_fastscan = false;
					} break;
					case CANOPEN_LSS_CMD_SWITCH_MODE: {
						if(self->lss.mute) break;
						// we only support turning of lss mode with this command
						if(self->lss.unlocked && msg->data[1] == 1) {
							canopen_debug("LSS: enabling\n");
							self->lss.enabled = true;
						} else if((self->lss.enabled || self->lss.unlocked) && msg->data[1] == 0) {
							self->lss.enabled = false;
							self->lss.mute = true; // this node will not process any more lss requests until lss is reset
							canopen_debug("LSS: muting enabled node\n");
						} else if(msg->data[1] == 0){
							// unmute fastscan if we are not the node this was intended for so we can respond to next scan
							self->lss.mute_fastscan = false;
							self->lss.part = 0;
							canopen_debug("LSS: unmuting fastscan\n");
						}
					} break;
					case CANOPEN_LSS_CMD_GET_ID: {
						if(!self->lss.enabled || self->lss.mute) break;
						// respond with node id
						struct can_message tmsg;
						can_message_init(&tmsg);
						tmsg.id = CANOPEN_COB_LSS | CANOPEN_LSS_RX;
						tmsg.len = 8;
						tmsg.data[0] = cmd;
						tmsg.data[1] = self->address;
						can_send(self->port, &tmsg, CANOPEN_CAN_TX_TIMEOUT);
					} break;
					case CANOPEN_LSS_CMD_SET_ID: {
						if(!self->lss.enabled || self->lss.mute) break;
						self->address = msg->data[1] & 0x7f;
						struct can_message tmsg;
						can_message_init(&tmsg);
						tmsg.id = CANOPEN_COB_LSS | CANOPEN_LSS_RX;
						tmsg.len = 8;
						tmsg.data[0] = cmd;
						can_send(self->port, &tmsg, CANOPEN_CAN_TX_TIMEOUT);
					} break;
					case CANOPEN_LSS_CMD_FASTSCAN: {
						// if lss is enabled on this node then we do not respond to fast scan
						if(self->lss.enabled || self->lss.mute || self->lss.mute_fastscan) {
							canopen_debug("LSS fastscan command ignored because lss is either already enabled or muted!\n");
							break;
						}

						uint32_t id = co32_to_u32(&msg->data[1]);
						uint8_t bit = msg->data[5];
						uint8_t part = msg->data[6];
						uint8_t next_part = msg->data[7];

						if(part > 3 || bit > 32 || next_part > 3){
							canopen_debug("Part or bit is out of range. Part: %d Next: %d Bits: %d\n", part, next_part, bit);
							break;
						}

						uint32_t myid = self->profile.identity[part];

						if(self->lss.part != part){
							canopen_debug("LSS parts do not match (%d %d). Need a reset!\n", self->lss.part, part);
							break;
						}

						uint32_t mask = 0xffffffff << bit;
						if((myid & mask) == (id & mask)){
							canopen_debug("LSS: confirming %d bits from left in %08x %08x (%08x) Masked: %08x %08x\n", 32 - bit, myid, id, mask, myid & mask, id & mask);
							struct can_message tmsg;
							can_message_init(&tmsg);
							tmsg.id = CANOPEN_COB_LSS | CANOPEN_LSS_RX;
							tmsg.len = 8;
							tmsg.data[0] = cmd;
							if(can_send(self->port, &tmsg, CANOPEN_CAN_TX_TIMEOUT) < 0){
								// TODO
								canopen_debug("Failed to send confirmation message\n");
								COVERAGE_DUMMY();
							}
						} else {
							canopen_debug("LSS: failed to confirm %d bits from left in %08x %08x (%08x) Masked: %08x %08x\n", 32 - bit, myid, id, mask, myid & mask, id & mask);
						}

						if(bit == 0 && part < 3){
							self->lss.part = next_part;
						} else if(bit == 0 && part == 3 && ((myid & 0xfffffffe) == (id & 0xfffffffe))){
							// currently last bit does not need to match. We really should find some good spec..
							self->lss.unlocked = true;
							canopen_debug("LSS is unlocked\n");
						} else if(bit == 0){
							self->lss.part = 0;
						}

						// if last bit in a part but for any part
						if(bit == 0){
							// if any of the previous bits do not match then we mute ourselves until next lss reset so we don't disturb other nodes
							uint32_t prev_mask = 0xfffffffe;
							if((myid & prev_mask) != (id & prev_mask)){
								canopen_debug("LSS: muting lss on slave due to last bit mismatch. %d %08x != %08x, (%08x %08x)\n", bit, myid & prev_mask, id & prev_mask, myid, id);
								self->lss.mute_fastscan = true;
							}
						}
					} break;
				}
			} else if(node_id == CANOPEN_LSS_RX) {
				// master mode, run state machine
				_handle_message_lss(self, msg);
			}
		} break;
	}
}

//static void _service_runner(struct work *work){
static void _service_runner(struct canopen *self){
	//struct canopen *self = container_of(work, struct canopen, work);

	// handle timeout of the sdo request
	if(self->sdo.req.running){
		timestamp_t time = micros();
		if(time_after(time, self->sdo.req.timeout)) {
			self->sdo.req.result = -ETIMEDOUT;
			self->sdo.req.running = false;
			thread_sem_give(&self->sdo.req.done);
		}
	}

	// send out any pending pdos
	_send_pending_pdos(self, PDO_SEND_ASYNC);

	// process timeouts associated with lss protocol
	_process_lss_timeouts(self);

	// send sync signal
	timestamp_t t = micros();
	if(self->mode == CANOPEN_MASTER && !(self->profile.sync_cob_id & CANOPEN_COB_DISABLED) && self->profile.cycle_period != 0 && time_after(t, self->sync_timeout)){
		self->sync_timeout = t + self->profile.cycle_period;

		canopen_send_sync(self);

		// send any local pdos that we have configured on the master
		_send_pending_pdos(self, PDO_SEND_SYNC);
	}

	// reschedule the work
	// TODO: set delay based on when next even should occur
	//queue_work(work, 0);
}
static void _can_tx(void *ptr){
	struct canopen *self = (struct canopen*)ptr;
	while(1){
		_service_runner(self);
		if(thread_sem_take_wait(&self->quit, 1) == 0){
			// this does the delay and if it's time to quit it quits right away without waiting
			break;
		}
	}
}
static ssize_t _read(struct vardir_entry_ops **ptr, struct vardir_entry *entry, uint32_t *value){
	struct canopen *self = container_of(ptr, struct canopen, var_ops);
	uint32_t id = entry->id & 0xffff00;
	uint32_t sub = entry->id & 0xff;
	int ret = 0;
	thread_mutex_lock(&self->lock);
	if(id == CANOPEN_REG_DEVICE_SERIAL) {
		if(sub >= 1 && sub <= 4){
			*value = self->profile.identity[(sub - 1) & 0x3];
		}
	} else if(id == CANOPEN_REG_DEVICE_SYNC_COB_ID) {
		*value = self->profile.sync_cob_id;
	} else if(id == CANOPEN_REG_DEVICE_CYCLE_PERIOD) {
		*value = self->profile.cycle_period;
	} else if(id >= 0x140000 && id <= 0x1AFF00){
		uint8_t pdo_id = (id >> 8) & 0x7f;
		if(0x140000 == (id & 0xff8000)){
			if(pdo_id > CANOPEN_DEFAULT_RXPDO_COUNT) {
				ret = -1;
				goto done;
			}
			struct canopen_pdo_entry *pdo = &self->profile.rxpdo[pdo_id];
			canopen_debug("PDO write settings for RXPDO %d\n", pdo_id);
			switch(sub){
				case 1: *value = pdo->cob_id; break;
				case 2: *value = pdo->type; break;
				case 3: *value = pdo->inhibit_time; break;
				case 4: *value = 0; break; // reserved
				case 5: *value = pdo->event_time; break;
			}
		} else if(0x160000 == (id & 0xff8000)){
			if(sub >= 8 || pdo_id > CANOPEN_DEFAULT_RXPDO_COUNT) {
				ret = -1;
				goto done;
			}
			struct canopen_pdo_entry *pdo = &self->profile.rxpdo[pdo_id];
			if(sub == 0) *value = pdo->map_entries;
			else *value = pdo->map[sub - 1];
		} else if(0x180000 == (id & 0xff8000)){
			if(pdo_id > CANOPEN_DEFAULT_TXPDO_COUNT) {
				ret = -1;
				goto done;
			}
			struct canopen_pdo_entry *pdo = &self->profile.txpdo[pdo_id];
			canopen_debug("PDO write settings for TXPDO %d\n", pdo_id);
			switch(sub){
				case 1: *value = pdo->cob_id; break;
				case 2: *value = pdo->type; break;
				case 3: *value = pdo->inhibit_time; break;
				case 4: *value = 0; break; // reserved
				case 5: *value = pdo->event_time; break;
			}
		} else if(0x1A0000 == (id & 0xff8000)){
			if(sub >= 8 || pdo_id > CANOPEN_DEFAULT_TXPDO_COUNT) {
				ret = -1;
				goto done;
			}
			struct canopen_pdo_entry *pdo = &self->profile.txpdo[pdo_id];
			if(sub == 0) *value = pdo->map_entries;
			else *value = pdo->map[sub - 1];
		}
	} else {
		ret = -ENOENT;
	}
done: 
	thread_mutex_unlock(&self->lock);
	return ret;
}

static ssize_t _write(struct vardir_entry_ops **ptr, struct vardir_entry *entry, uint32_t value){
	struct canopen *self = container_of(ptr, struct canopen, var_ops);
	uint32_t id = entry->id & 0xffff00;
	uint32_t sub = entry->id & 0xff;
	int ret = 0;
	thread_mutex_lock(&self->lock);
	if(id == CANOPEN_REG_DEVICE_SYNC_COB_ID) {
		self->profile.sync_cob_id = value;
	} else if(id == CANOPEN_REG_DEVICE_CYCLE_PERIOD) {
		self->profile.cycle_period = value;
	} else if(id >= 0x140000 && id <= 0x1AFF00) {
		uint8_t pdo_id = (id >> 8) & 0x7f;
		if(0x140000 == (id & 0xff8000)){
			if(pdo_id > CANOPEN_DEFAULT_RXPDO_COUNT){
				ret = -1;
				goto done;
			}
			struct canopen_pdo_entry *pdo = &self->profile.rxpdo[pdo_id];
			switch(sub){
				case 1: pdo->cob_id = value; break;
				case 2: pdo->type = (uint8_t)value; break;
				case 3: pdo->inhibit_time = (uint16_t)value; break;
				case 4: break; // reserved
				case 5: pdo->event_time = (uint16_t)value; break;
			}
		} else if(0x160000 == (id & 0xff8000)){
			if(sub > 8 || pdo_id > CANOPEN_DEFAULT_RXPDO_COUNT) {
				ret = -1;
				goto done;
			}
			struct canopen_pdo_entry *pdo = &self->profile.rxpdo[pdo_id];
			if(sub == 0) pdo->map_entries = (uint8_t)value;
			else pdo->map[sub - 1] = value;
		} else if(0x180000 == (id & 0xff8000)){
			if(pdo_id > CANOPEN_DEFAULT_TXPDO_COUNT) {
				ret = -1;
				goto done;
			}
			struct canopen_pdo_entry *pdo = &self->profile.txpdo[pdo_id];
			switch(sub){
				case 1: pdo->cob_id = value; break;
				case 2: pdo->type = (uint8_t)value; break;
				case 3: pdo->inhibit_time = (uint16_t)value; break;
				case 4: break; // reserved
				case 5: pdo->event_time = (uint16_t)value; break;
			}
		} else if(0x1A0000 == (id & 0xff8000)){
			if(sub > 8 || pdo_id > CANOPEN_DEFAULT_TXPDO_COUNT) {
				ret = -1;
				goto done;
			}
			struct canopen_pdo_entry *pdo = &self->profile.txpdo[pdo_id];
			if(sub == 0) pdo->map_entries = (uint8_t)value;
			else pdo->map[sub - 1] = value;
		}
	} else {
		ret = -ENOENT;
		goto done;
	}
done:
	thread_mutex_unlock(&self->lock);
	return ret;
}

static struct vardir_entry_ops _canopen_ops = {
	.read = _read,
	.write = _write
};

void canopen_init(struct canopen *self, struct vardir *directory, can_port_t can, canopen_mode_t mode){
	memset(self, 0, sizeof(*self));

	canopen_debug("struct size %lu, profile offset %lu, sizeof(work) %lu\n", sizeof(struct canopen), offsetof(struct canopen, profile), sizeof(struct work));

	thread_mutex_init(&self->mx);
	thread_sem_init(&self->quit);
	self->port = can;
	self->mode = mode;
	self->vardir = directory;
	self->var_ops = &_canopen_ops;
	INIT_LIST_HEAD(&self->emcy_listeners);

	self->profile.sync_cob_id = CANOPEN_COB_SYNC_OR_EMCY;

	vardir_add_field(self->vardir,	CANOPEN_REG_DEVICE_TYPE,		NULL, VAR_UINT32 | VAR_CONSTANT, 402);
	vardir_add_entry(self->vardir,	CANOPEN_REG_DEVICE_SYNC_COB_ID,	NULL, VAR_UINT32, &self->var_ops);
	vardir_add_entry(self->vardir,	CANOPEN_REG_DEVICE_CYCLE_PERIOD,NULL, VAR_UINT32, &self->var_ops);

	vardir_add_field(self->vardir,	CANOPEN_REG_DEVICE_SERIAL,		NULL, VAR_UINT8 | VAR_CONSTANT, 4);
	vardir_add_entry(self->vardir,	CANOPEN_REG_DEVICE_SERIAL + 1,	"device_id[0]", VAR_UINT32, &self->var_ops);
	vardir_add_entry(self->vardir,	CANOPEN_REG_DEVICE_SERIAL + 2,	"device_id[1]", VAR_UINT32, &self->var_ops);
	vardir_add_entry(self->vardir,	CANOPEN_REG_DEVICE_SERIAL + 3,	"device_id[2]", VAR_UINT32, &self->var_ops);
	vardir_add_entry(self->vardir,	CANOPEN_REG_DEVICE_SERIAL + 4,	"device_id[3]", VAR_UINT32, &self->var_ops);

	const uint32_t base[4] = { 0x140000, CANOPEN_DEFAULT_RXPDO_COUNT, 0x180000, CANOPEN_DEFAULT_TXPDO_COUNT};
	for(int u = 0; u < 2; u++){
		uint32_t count = base[u * 2 + 1];
		uint32_t base_addr = base[u * 2];
		for(uint32_t c = 0; c < count; c++){
			uint32_t config_base = (uint32_t)(base_addr + (uint32_t)(c << 8));
			uint32_t map_base = (uint32_t)(base_addr + 0x020000 + (uint32_t)(c << 8));
			vardir_add_field(self->vardir,	config_base,	NULL, VAR_UINT8 | VAR_CONSTANT, 5);
			vardir_add_entry(self->vardir,	config_base + 1,NULL, VAR_UINT32, &self->var_ops);
			vardir_add_entry(self->vardir,	config_base + 2,NULL, VAR_UINT8, &self->var_ops);
			vardir_add_entry(self->vardir,	config_base + 3,NULL, VAR_UINT16, &self->var_ops);
			vardir_add_entry(self->vardir,	config_base + 4,NULL, VAR_UINT16, &self->var_ops);
			vardir_add_entry(self->vardir,	config_base + 5,NULL, VAR_UINT16, &self->var_ops);
			vardir_add_entry(self->vardir,	map_base,		NULL, VAR_UINT8, &self->var_ops);
			vardir_add_entry(self->vardir,	map_base + 1,	NULL, VAR_UINT32, &self->var_ops);
			vardir_add_entry(self->vardir,	map_base + 2,	NULL, VAR_UINT32, &self->var_ops);
			vardir_add_entry(self->vardir,	map_base + 3,	NULL, VAR_UINT32, &self->var_ops);
			vardir_add_entry(self->vardir,	map_base + 4,	NULL, VAR_UINT32, &self->var_ops);
		}
	}

	thread_mutex_init(&self->lock);

	thread_mutex_init(&self->lss.mx);
	thread_sem_init_counting(&self->lss.req.done, 1, 0);

	thread_mutex_init(&self->sdo.mx);
	thread_sem_init_counting(&self->sdo.req.done, 1, 0);

	can_listener_init(&self->listener, _handle_message);
	can_register_listener(self->port, &self->listener);

	thread_create(_can_tx, "can_tx", 230, self, 3, &self->task);

	// start the work scheduler
	//work_init(&self->work, _service_runner);
	//queue_work(&self->work, 0);
}

void canopen_set_identity(struct canopen *self, uint32_t identity[4]){
	thread_mutex_lock(&self->lock);
	memcpy(self->profile.identity, identity, sizeof(uint32_t) * 4);
	thread_mutex_unlock(&self->lock);
}

void canopen_set_node_id(struct canopen *self, uint8_t id){
	thread_mutex_lock(&self->lock);
	self->address = id;
	thread_mutex_unlock(&self->lock);
}

void canopen_set_sync_period(struct canopen *self, uint32_t period_us){
	thread_mutex_lock(&self->lock);
	self->profile.cycle_period = constrain_u32((uint32_t)(period_us - 1), 0, 1000000);
	// schedule a sync right away
	self->sync_timeout = micros();
	thread_mutex_unlock(&self->lock);
}

void canopen_destroy(struct canopen *self){
	thread_sem_give(&self->quit);
	thread_join(self->task);
	thread_sem_destroy(&self->lss.req.done);
	thread_sem_destroy(&self->sdo.req.done);
	thread_mutex_destroy(&self->mx);
	thread_sem_destroy(&self->quit);
	thread_mutex_destroy(&self->lss.mx);
	thread_mutex_destroy(&self->sdo.mx);
}

int canopen_lss_find_node(struct canopen *self, struct canopen_serial_number *serial){
	if(!serial) return -EINVAL;

	// this will just unmute fastscan on nodes that do not have lss enabled and fully mute the node that has lss enabled
	canopen_lss_mode(self, 0);

	// lock lss mutex because only one lss request can be active on the network at a time
	thread_mutex_lock(&self->lss.mx);

	canopen_debug("LSS master: starting scan\n");

	self->lss.req.cmd = CANOPEN_LSS_CMD_FASTSCAN;
	self->lss.req.running = true;
	self->lss.req.result = 0;
	self->lss.part = 0;
	self->lss.bit = 31;
	self->lss.id = 0;
	self->lss.bits_per_test = 1;
	memset(self->lss.req.serial, 0, sizeof(self->lss.req.serial));

	_lss_scan_request(self);

	// wait until the request either completes or fails
	thread_sem_take(&self->lss.req.done);

	if(self->lss.req.result == 0){
		memcpy(serial->part, self->lss.req.serial, 4 * sizeof(uint32_t));
	} else {
		memset(serial->part, 0, 4 * sizeof(uint32_t));
	}

	self->lss.req.running = false;
	int r = self->lss.req.result;

	// release the lss subsystem
	thread_mutex_unlock(&self->lss.mx);

	return r;
}

int canopen_lss_enable(struct canopen *self, struct canopen_serial_number *serial){
	if(!serial) return -EINVAL;

	// we always reset lss (this also resets any scan that is in progress)
	// we need to do this in order to ensure that we can start a new ident sequence
	canopen_lss_reset(self);

	// lock lss mutex because only one lss request can be active on the network at a time
	thread_mutex_lock(&self->lss.mx);

	canopen_debug("LSS master: enabling lss on specific node\n");

	self->lss.req.running = true;
	self->lss.req.cmd = CANOPEN_LSS_CMD_FASTSCAN;
	self->lss.req.result = 0;
	self->lss.part = 0;
	self->lss.bit = 0;
	self->lss.bits_per_test = 31;
	self->lss.id = serial->part[0];
	memcpy(self->lss.req.serial, serial->part, 4 * sizeof(uint32_t));

	_lss_scan_request(self);

	// wait until the request either completes or fails
	thread_sem_take(&self->lss.req.done);

	canopen_lss_mode(self, 1);

	int r = self->lss.req.result;

	// release the lss subsystem
	thread_mutex_unlock(&self->lss.mx);

	return r;

}

int canopen_lss_set_node_id(struct canopen *self, struct canopen_serial_number *serial, uint8_t node_id){
	if(!serial || node_id > 0x7f) return -EINVAL;

	// reset and enable lss on target node
	canopen_lss_enable(self, serial);

	// lock lss mutex because only one lss request can be active on the network at a time
	thread_mutex_lock(&self->lss.mx);
	self->lss.req.cmd = CANOPEN_LSS_CMD_SET_ID;
	self->lss.req.node_id = node_id;
	self->lss.req.result = 0;
	memcpy(self->lss.req.serial, serial->part, 4 * sizeof(uint32_t));

	self->lss.timeout = micros() + CANOPEN_LSS_TIMEOUT_MS * 1000U;
	self->lss.state = CANOPEN_LSS_STATE_SET_ID_WAIT_CONFIRM;
	self->lss.req.running = true;

	// send request
	canopen_debug("LSS: setting id\n");
	struct can_message msg;
	can_message_init(&msg);
	msg.id = CANOPEN_COB_LSS | CANOPEN_LSS_TX;
	msg.len = 8;
	msg.data[0] = CANOPEN_LSS_CMD_SET_ID;
	msg.data[1] = self->lss.req.node_id & 0x7f;

	if(can_send(self->port, &msg, CANOPEN_CAN_TX_TIMEOUT) < 0){
		// TODO: handle error
		COVERAGE_DUMMY();
	}

	// wait until the request either completes or fails
	thread_sem_take(&self->lss.req.done);
	self->lss.req.running = false;

	// go back to normal mode
	canopen_lss_mode(self, 0);

	int r = self->lss.req.result;

	// release the lss subsystem
	thread_mutex_unlock(&self->lss.mx);

	return r;
}

int canopen_sdo_read(struct canopen *self, uint8_t node_id, uint32_t dict, void *ptr, size_t size){
	if(size > 4) return -CANOPEN_SDO_ERR_LEN_HIGH;
	uint8_t *data = (uint8_t*)ptr;

	// lock lss mutex because only one lss request can be active on the network at a time
	thread_mutex_lock(&self->sdo.mx);

	self->sdo.req.cmd = CANOPEN_SDO_CMD_READ;
	self->sdo.req.node_id = node_id;
	self->sdo.req.id = dict;
	self->sdo.req.output = data;
	self->sdo.req.output_len = size;
	self->sdo.req.timeout = micros() + CANOPEN_SDO_TIMEOUT_MS * 1000U;
	self->sdo.req.result = 0;

	// start the request
	self->sdo.req.running = true;

	struct can_message tmsg;
	can_message_init(&tmsg);
	tmsg.id = (uint32_t)(CANOPEN_COB_RXSDO | node_id);
	tmsg.len = 8;
	// TODO: support multipacket transfers
	switch(size){
		case 1: tmsg.data[0] = CANOPEN_SDO_CMD_READ1; break;
		case 2: tmsg.data[0] = CANOPEN_SDO_CMD_READ2; break;
		case 3: tmsg.data[0] = CANOPEN_SDO_CMD_READ3; break;
		case 4: tmsg.data[0] = CANOPEN_SDO_CMD_READ4; break;
		default: return -CANOPEN_SDO_ERR_LEN_HIGH;
	};
	u16_to_co16((uint16_t)(dict >> 8), &tmsg.data[1]);
	tmsg.data[3] = (uint8_t)(dict & 0xff);
	if(can_send(self->port, &tmsg, CANOPEN_CAN_TX_TIMEOUT) < 0){
		thread_mutex_unlock(&self->sdo.mx);
		return -EIO;
	}

	// wait until the request either completes or fails
	while(self->sdo.req.running){
		thread_sem_take(&self->sdo.req.done);
	}

	self->sdo.req.running = false;

	int r = self->sdo.req.result;

	// release the lss subsystem
	thread_mutex_unlock(&self->sdo.mx);

	return r;
}

int canopen_sdo_write(struct canopen *self, uint8_t node_id, uint32_t dict, const uint8_t *data, size_t size){
	// lock lss mutex because only one lss request can be active on the network at a time
	thread_mutex_lock(&self->sdo.mx);

	struct can_message tmsg;
	can_message_init(&tmsg);

	tmsg.id = (uint32_t)(CANOPEN_COB_RXSDO | node_id);
	tmsg.len = 8;
	switch(size){
		case 1: tmsg.data[0] = CANOPEN_SDO_CMD_WRITE1; break;
		case 2: tmsg.data[0] = CANOPEN_SDO_CMD_WRITE2; break;
		case 3: tmsg.data[0] = CANOPEN_SDO_CMD_WRITE3; break;
		case 4: tmsg.data[0] = CANOPEN_SDO_CMD_WRITE4; break;
		default: return -EINVAL;
	}
	u16_to_co16((uint16_t)(dict >> 8), &tmsg.data[1]);
	tmsg.data[3] = (uint8_t)(dict & 0xff);
	memcpy(&tmsg.data[4], data, size);

	self->sdo.req.cmd = CANOPEN_SDO_CMD_WRITE;
	self->sdo.req.node_id = node_id;
	self->sdo.req.id = dict;
	self->sdo.req.timeout = micros() + CANOPEN_SDO_TIMEOUT_MS * 1000U;
	self->sdo.req.result = -1;
	self->sdo.req.running = true;

	if(can_send(self->port, &tmsg, CANOPEN_CAN_TX_TIMEOUT) < 0){
		thread_mutex_unlock(&self->sdo.mx);
		return -EIO;
	}

	// wait until the request either completes or fails
	while(self->sdo.req.running){
		thread_sem_take(&self->sdo.req.done);
	}
	self->sdo.req.running = false;

	int r = self->sdo.req.result;

	// release the lss subsystem
	thread_mutex_unlock(&self->sdo.mx);

	return r;
}

int canopen_sdo_write_u32(struct canopen *self, uint8_t node_id, uint32_t dic, uint32_t value){
	uint8_t data[4] = {0, 0, 0, 0};
	u32_to_co32(value, data);
	return canopen_sdo_write(self, node_id, dic, data, 4);
}

int canopen_sdo_write_i32(struct canopen *self, uint8_t node_id, uint32_t dic, int32_t value){
	return canopen_sdo_write_u32(self, node_id, dic, (uint32_t)value);
}

int canopen_sdo_write_u16(struct canopen *self, uint8_t node_id, uint32_t dic, uint16_t value){
	uint8_t data[2] = {0, 0};
	u16_to_co16((uint32_t)value, data);
	return canopen_sdo_write(self, node_id, dic, data, 2);
}

int canopen_sdo_write_i16(struct canopen *self, uint8_t node_id, uint32_t dic, int16_t value){
	return canopen_sdo_write_u16(self, node_id, dic, (uint16_t)value);
}

int canopen_sdo_write_u8(struct canopen *self, uint8_t node_id, uint32_t dic, uint8_t value){
	return canopen_sdo_write(self, node_id, dic, &value, 1);
}

int canopen_sdo_write_i8(struct canopen *self, uint8_t node_id, uint32_t dic, int8_t value){
	return canopen_sdo_write(self, node_id, dic, (uint8_t*)&value, 1);
}

int canopen_sdo_read_u32(struct canopen *self, uint8_t node_id, uint32_t dic, uint32_t *value){
	uint8_t data[4] = {0, 0, 0, 0};
	int ret;
	if((ret = canopen_sdo_read(self, node_id, dic, data, 4)) < 0){
		return ret;
	}
	*value = co32_to_u32(data);
	return 0;
}

int canopen_sdo_read_i32(struct canopen *self, uint8_t node_id, uint32_t dic, int32_t *value){
	uint8_t data[4] = {0, 0, 0, 0};
	int ret;
	if((ret = canopen_sdo_read(self, node_id, dic, data, 4)) < 0){
		return ret;
	}
	*value = (int32_t)co32_to_u32(data);
	return 0;
}

int canopen_sdo_read_u16(struct canopen *self, uint8_t node_id, uint32_t dic, uint16_t *value){
	uint8_t data[2] = {0, 0};
	int ret;
	if((ret = canopen_sdo_read(self, node_id, dic, data, 2)) < 0){
		return ret;
	}
	*value = co16_to_u16(data);
	return 0;

}

int canopen_sdo_read_i16(struct canopen *self, uint8_t node_id, uint32_t dic, int16_t *value){
	uint8_t data[2] = {0, 0};
	int ret;
	if((ret = canopen_sdo_read(self, node_id, dic, data, 2)) < 0){
		return ret;
	}
	*value = (int16_t)co16_to_u16(&data[0]);
	return 0;
}

int canopen_sdo_read_u8(struct canopen *self, uint8_t node_id, uint32_t dic, uint8_t *value){
	return canopen_sdo_read(self, node_id, dic, (char*)value, 1);
}

int canopen_sdo_read_i8(struct canopen *self, uint8_t node_id, uint32_t dic, int8_t *value){
	return canopen_sdo_read(self, node_id, dic, (char*)value, 1);
}

int canopen_sdo_read_device_type(struct canopen *self, uint8_t node_id, struct canopen_device_type *type){
	uint32_t dev_type = 0;
	if(canopen_sdo_read_u32(self, node_id, 0x100000, &dev_type) < 0){
		return -1;
	}
	static struct {
		uint16_t profile;
		const char *name;
	} _type[] = {
		{ 401, "Generic I/O Module" },
		{ 402, "Drive or Motion Control device" },
		{ 404, "Measuring device or Closed Loop Controller" },
		{ 405, "IEC 61131-3 Programmable Devices" },
		{ 406, "Rotating or Linear Encoder" },
		{ 408, "Hydraulic Drive or Proportional Valve" },
		{ 410, "Inclinometer" },
		{ 412, "Medical Device"},
		{ 413, "Truck Gateway"}, 
		{ 414, "Yarn Feeding Unit (Weaving Machine)"},
		{ 415, "Road Construction Machinery"}, 
		{ 416, "Building Door Control"}, 
		{ 417, "Lift Control System"}, 
		{ 418, "Battery Module"}, 
		{ 419, "Battery Charger"}, 
		{ 420, "Extruder Downstream Device"}, 
		{ 422, "Municipal Vehicle – CleANopen"}, 
		{ 423, "Railway Diesel Control System"}, 
		{ 424, "Rail Vehicle Door Control System"}, 
		{ 425, "Medical Diagnostic Add-on Module"}, 
		{ 445, "RFID Device"},
	};

	uint16_t profile = dev_type & 0xffff;
	for(size_t t = 0; t < (sizeof(_type) / sizeof(_type[0])); t++){
		if(_type[t].profile == profile){
			type->name = _type[t].name;
			type->profile = _type[t].profile;
			return 0;
		}
	}

	type->name = "Not found";
	type->profile = 0;
	return -1;
}

int canopen_sdo_read_serial(struct canopen *self, uint8_t node_id, struct canopen_serial_number *serial){
	uint32_t len = 0;
	if(canopen_sdo_read_u32(self, node_id, CANOPEN_REG_DEVICE_SERIAL, &len) < 0){
		return -1;
	}
	if(len != 4){
		return -1;
	}

	for(unsigned int c = 0; c < 4; c++){
		if(canopen_sdo_read_u32(self, node_id, CANOPEN_REG_DEVICE_SERIAL + c + 1, &serial->part[c]) < 0){
			return -EIO;
		}
	}

	return 0;
}

static int _canopen_pdo_configure(struct canopen *self, uint32_t base, uint8_t node_id, const struct canopen_pdo_config *conf){
	int ret = 0;
	uint32_t index = (uint32_t)conf->index << 8;
	uint8_t num_entries = 0;

	// read number of entries in the table
	if((ret = canopen_sdo_read_u8(self, node_id, (uint32_t)(base + index), &num_entries)) < 0){
		return ret;
	}

	// disable tx pdo
	if((ret = canopen_sdo_write_u32(self, node_id, (uint32_t)(base + index + 1), conf->cob_id | CANOPEN_PDO_DISABLED)) < 0){
		return ret;
	}

	// update pdo type
	if((ret = canopen_sdo_write_u8(self, node_id, (uint32_t)(base + index + 2), conf->type)) < 0){
		return ret;
	}

	if(num_entries > 2 && (ret = canopen_sdo_write_u16(self, node_id, (uint32_t)(base + index + 3), conf->inhibit_time)) < 0){
		return ret;
	}

	if(num_entries > 4 && (ret = canopen_sdo_write_u16(self, node_id, (uint32_t)(base + index + 5), conf->event_time)) < 0){
		return ret;
	}

	// disable mapping first
	if((ret = canopen_sdo_write_u8(self, node_id, (uint32_t)(base + 0x020000 + index), 0)) < 0){
		return ret;
	}

	uint8_t num = 0;
	for(size_t c = 0; c < (sizeof(conf->map) / sizeof(conf->map[0])); c++){
		if(conf->map[c] == 0) break;
		// write mapping of the tx pdo
		if((ret = canopen_sdo_write_u32(self, node_id, (uint32_t)(base + 0x020000 + index + c + 1), conf->map[c])) < 0){
			return ret;
		}
		num++;
	}

	// write number of mapping entries
	if((ret = canopen_sdo_write_u8(self, node_id, (uint32_t)(base + 0x020000 + index), num)) < 0){
		return ret;
	}

	// enable the tx pdo and set cob id
	if((ret = canopen_sdo_write_u32(self, node_id, (uint32_t)(base + index + 1), conf->cob_id)) < 0){
		return ret;
	}
	return ret;
}

int canopen_pdo_tx_local(struct canopen *self, const struct canopen_pdo_config *conf){
	size_t max_entries = CANOPEN_DEFAULT_TXPDO_COUNT;
	size_t max_map = CANOPEN_DEFAULT_PDO_MAP_COUNT;
	if(conf->index >= max_entries) return -EINVAL;
	struct canopen_pdo_entry *e = &self->profile.txpdo[conf->index];
	e->cob_id = conf->cob_id | CANOPEN_COB_DISABLED;
	e->type = conf->type;
	e->inhibit_time = conf->inhibit_time;
	e->event_time = conf->event_time;
	e->map_entries = 0;
	for(size_t c = 0; c < max_map; c++){
		if(conf->map[c] == 0) break;
		e->map[c] = conf->map[c];
		e->map_entries++;
	}
	e->sync_cycles = 0;
	e->cob_id = conf->cob_id;
	return 0;
}

int canopen_pdo_rx_local(struct canopen *self, const struct canopen_pdo_config *conf){
	size_t max_entries = CANOPEN_DEFAULT_RXPDO_COUNT;
	size_t max_map = CANOPEN_DEFAULT_PDO_MAP_COUNT;
	if(conf->index >= max_entries) return -EINVAL;
	struct canopen_pdo_entry *e = &self->profile.rxpdo[conf->index];
	e->cob_id = conf->cob_id | CANOPEN_COB_DISABLED;
	e->type = conf->type;
	e->inhibit_time = conf->inhibit_time;
	e->event_time = conf->event_time;
	e->map_entries = 0;
	for(size_t c = 0; c < max_map; c++){
		if(conf->map[c] == 0) break;
		e->map[c] = conf->map[c];
		e->map_entries++;
	}
	e->sync_cycles = 0;
	e->cob_id = conf->cob_id;
	return 0;
}

int canopen_pdo_rx(struct canopen *self, uint8_t node_id, const struct canopen_pdo_config *conf){
	if(self->mode != CANOPEN_MASTER) return -1;
	return _canopen_pdo_configure(self, 0x140000, node_id, conf);
}

int canopen_pdo_tx(struct canopen *self, uint8_t node_id, const struct canopen_pdo_config *conf){
	if(self->mode != CANOPEN_MASTER) return -1;
	return _canopen_pdo_configure(self, 0x180000, node_id, conf);
}

int canopen_pdo_transmit(struct canopen *self, uint16_t cob_id, const uint8_t data[8]){
	if(!self->address) return -1;
	uint32_t pdo_id = (uint32_t)(cob_id & 0x7ff);
	if(pdo_id < 0x180 || pdo_id >= 0x580) return -EINVAL;
	struct can_message msg;
	can_message_init(&msg);
	msg.id = pdo_id;
	msg.len = 8;
	memcpy(msg.data, data, 8);
	return can_send(self->port, &msg, CANOPEN_CAN_TX_TIMEOUT);
}

int canopen_send_sync(struct canopen *self){
	if(self->mode != CANOPEN_MASTER) return -EPERM;
	struct can_message msg;
	can_message_init(&msg);
	msg.id = (self->profile.sync_cob_id)?self->profile.sync_cob_id:CANOPEN_COB_SYNC_OR_EMCY;
	msg.len = 0;
	if(can_send(self->port, &msg, CANOPEN_CAN_TX_TIMEOUT) < 0) return -EIO;
	return 0;
}

int canopen_nmt_send(struct canopen *self, uint8_t node_id, uint8_t command){
	if(self->mode != CANOPEN_MASTER) return -EPERM;
	struct can_message msg;
	can_message_init(&msg);
	msg.id = 0;
	msg.len = 2;
	msg.data[0] = command;
	msg.data[1] = node_id;
	if(can_send(self->port, &msg, CANOPEN_CAN_TX_TIMEOUT) < 0) return -EIO;
	return 0;
}

int canopen_nmt_reset(struct canopen *self, uint8_t node_id){
	return canopen_nmt_send(self, node_id, CANOPEN_NMT_CMD_RESET);
}

int canopen_nmt_enable(struct canopen *self, uint8_t node_id){
	return canopen_nmt_send(self, node_id, CANOPEN_NMT_CMD_ENABLE);
}

void canopen_listener_init(struct canopen_listener *self, void (*callback)(struct canopen_listener *self, uint8_t node_id, struct can_message *msg)){
	memset(self, 0, sizeof(*self));
	INIT_LIST_HEAD(&self->list);
	self->callback = callback;
}

void canopen_register_listener(struct canopen *self, struct canopen_listener *l){
	thread_mutex_lock(&self->mx);
	list_add_tail(&l->list, &self->emcy_listeners);
	thread_mutex_unlock(&self->mx);
}

const char *canopen_strerror(int32_t err){
	// usually error is a negative value, flip the sign
	if(err < 0) err = -err;

	// SDO errors
	if(err == 0x05030000) return "Toggle bit not altered";
	else if(err == 0x05040001) return "Client/server command specifier not valid or unknown";
	else if(err == 0x06010000) return "Unsupported access to an object";
	else if(err == 0x06020000) return "Object does not exist in object directory";
	else if(err == 0x06040041) return "Object can not be mapped to the PDO";
	else if(err == 0x06040042) return "Number of objects to be mapped would exceed pdo length";
	else if(err == 0x06040043) return "SDO parameters incompatible";
	else if(err == 0x06040047) return "General internal incompatibility reason in device";
	else if(err == 0x06070010) return "Data type does not match, length of service parameter does not match";
	else if(err == 0x06070012) return "Data type does not match, length of service parameter too high";
	else if(err == 0x06070013) return "Data type does not match, length of service parameter too low";
	else if(err == 0x06090011) return "Sub-index does not exist";
	else if(err == 0x06090030) return "Value range of parameter exceeded (only for write access)";
	else if(err == 0x06090031) return "Value of parameter written too high";
	else if(err == 0x06090032) return "Value of parameter written too low";
	else if(err == 0x08000000) return "General error";
	else if(err == 0x08000020) return "Data cannot be transferred or stored to the application";
	else if(err == 0x08000021) return "Data cannot be transferred or stored to the application because of local control";
	else if(err == 0x08000022) return "Data cannot be transferred or stored to the application because of the present device state";

	const char *serr = strerror(err);
	if(serr) return serr;

	return "Unknown canopen error";
}
