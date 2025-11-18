/*
 * distance_buffer.h
 *
 *  Created on: Sep 30, 2025
 *      Author: jorgelarach
 */

#ifndef DISTANCE_BUFFER_H_
#define DISTANCE_BUFFER_H_


#include <stdint.h>
#include <stddef.h> // for size_t

#ifdef __cplusplus
extern "C" {
#endif

#define DBUF_SIZE 3  // Change here for number of recent readings

typedef struct{
	float buffer[DBUF_SIZE];
	size_t head;
	size_t count;
} DBUF;

void DBUF_init(DBUF *db);
void DBUF_add(DBUF *db, float distance_sample);
float DBUF_average(DBUF *db);

#ifdef __cplusplus
}
#endif

#endif /* DISTANCE_BUFFER_H_ */
