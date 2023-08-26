#include "util.hpp"

char *dump_bytes(char *buf, const uint8_t *p, uint8_t count)
{
	const uint8_t *src = p;
	char *dest = buf;
	while (count-- > 0)
	{
		dest += sprintf(dest, "%02X", *src++);
		if (count)
			*dest++ = ' ';
	}
	*dest = '\0'; // return an empty sting for an empty memory chunk
	return buf;
}