#ifndef _FOP_H
#define _FOP_H

#undef sext
#define sext(x) (x & 0x800000 ? x | 0xFF000000 : x & 0x00FFFFFF)

static size_t float_init_wav(FILE *fp) 
{
	char header[] = {
		0x52, 0x49, 0x46, 0x46, /* "RIFF" */
		0x00, 0x00, 0x00, 0x00, /* Size */
		0x57, 0x41, 0x56, 0x45, /* "WAVE" */
		0x66, 0x6D, 0x74, 0x20, /* "fmt " */
		0x10, 0x00, 0x00, 0x00, /* size of fmt chunk (16) */
		0x03, 0x00, 0x01, 0x00, /* float; 1-channel */
		0x44, 0xAC, 0x00, 0x00, /* 44100 Hz */	
		0x20, 0x62, 0x05, 0x00, /* Byte Rate */
		0x08, 0x00, 0x40, 0x00, /* 8 Bytes Align, 64 bits per sample */
		0x66, 0x61, 0x63, 0x74, /* "fact" */
		0x04, 0x00, 0x00, 0x00,
		0xA8, 0xBA, 0x06, 0x00,
		0x50, 0x45, 0x41, 0x4B, /* "PEAK" */
		0x10, 0x00, 0x00, 0x00,
		0x01, 0x00, 0x00, 0x00,
		0xFB, 0xA7, 0xAB, 0x53,
		0xA6, 0x30, 0x33, 0x3F,
		0x7D, 0xCA, 0x05, 0x00,
		0x64, 0x61, 0x74, 0x61, /* "data" */
		0x00, 0x00, 0x00, 0x00, /* Data chunk size (Size - 0x24) */
	};

	return fwrite(header, 1, sizeof(header), fp);
}

static size_t int_init_wav(FILE *file, int stereo)
{
	/* Create header */
	char sample_header[] = {
		0x52, 0x49, 0x46, 0x46, /* "RIFF" */
		0x00, 0x00, 0x00, 0x00, /* Size */
		0x57, 0x41, 0x56, 0x45, /* "WAVE" */
		0x66, 0x6D, 0x74, 0x20, /* "fmt " */
		0x10, 0x00, 0x00, 0x00, /* size of fmt chunk (16) */
		0x01, 0x00, 0x02, 0x00, /* PCM; 2-channels */
		0x44, 0xAC, 0x00, 0x00, /* 44100 Hz */	
		0x98, 0x09, 0x04, 0x00, /* Byte Rate */
		0x06, 0x00, 0x18, 0x00, /* 6 Bytes Align, 24 bits per sample */
		0x64, 0x61, 0x74, 0x61, /* "data" */
		0x00, 0x00, 0x00, 0x00, /* Data chunk size (Size - 0x24) */
	};

	if (!stereo) {
		sample_header[22] = 0x01;
		sample_header[28] = 0xCC;
		sample_header[29] = 0x04;
		sample_header[30] = 0x02;
		sample_header[32] = 0x03;
	}

	return fwrite(sample_header, 1, sizeof(sample_header), file);
}

static int float_finalize_wav(FILE *file, unsigned int filesize) 
{
	filesize -= 8;
	if (filesize < 0) return;

	printf("size: %d\n", filesize);
	fseek(file, 0x4, SEEK_SET);
	fwrite(&filesize, 4, 1, file);
	filesize += 0x48; // header size
	fseek(file, 0x4C, SEEK_SET);
	fwrite(&filesize, 4, 1, file);
}

static int int_finalize_wav(FILE *file, unsigned int filesize) 
{
	filesize -= 4;
	if (filesize < 0) return;

	printf("size: %d\n", filesize);
	fseek(file, 0x4, SEEK_SET);
	fwrite(&filesize, 4, 1, file);
	filesize += 0x24; // header size
	fseek(file, 0x28, SEEK_SET);
	fwrite(&filesize, 4, 1, file);
}

static inline double fix2fl(int s) 
{
	const double Q = 1.0F / (0x00000000007fffff + 0.5);
	return (double) s * Q;
}

static inline int fl2fix(double s)
{
	return (int) ceil(s * (0x00000000007fffffF + 0.5));
}

static inline int fl2fix26(double s)
{
	//return (int) ceil(s * (0x000000001ffffffF + 0.5));
	return (int) ceil(s * (0x0000000003fffffF + 0.5));
}

#endif
