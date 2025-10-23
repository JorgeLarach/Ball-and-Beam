/*
 * db.c
 *
 *  Created on: Sep 30, 2025
 *      Author: jorgelarach
 */

#include "distance_buffer.h"

void DBUF_init(DBUF *db){
	db->head = 0;
	db->count = 0;
	for(size_t i = 0; i < DBUF_SIZE; i++){
		db->buffer[i] = 0.0f;
	}
}

void DBUF_add(DBUF *db, float distance_sample){
	db->buffer[db->head] = distance_sample;
	db->head = (db->head + 1) % DBUF_SIZE;
	if(db->count < DBUF_SIZE) db->count++;
}

float DBUF_average(DBUF *db){
	if(db->count == 0) return 0.0f;
	float sum = 0.0f;
	for(size_t i = 0; i < db->count; i++){
		sum += db->buffer[i];
	}
	return sum / db->count;
}
