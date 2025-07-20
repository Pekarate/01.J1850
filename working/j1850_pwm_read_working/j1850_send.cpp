byte crc(byte *msg_buf, int nbytes) {
	byte crc = 0xFF;
	while (nbytes--) {
		crc ^= *msg_buf++;
		for (int i = 0; i < 8; i++)
			crc = crc & 0x80 ? (crc << 1) ^ 0x1D : crc << 1;
	}
	crc ^= 0xFF;
	return crc;
}