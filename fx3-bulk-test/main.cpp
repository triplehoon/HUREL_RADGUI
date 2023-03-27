#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include <pthread.h>

#include "cyusb.h"

#include "cruxellIO.h"
#include "test.h"

static const char *program_name;
static const char *const short_options = "hvt:";
static const struct option long_options[] = {
	{"help", 0, NULL, 'h'},
	{"version", 0, NULL, 'v'},
	{
		"timeout",
		1,
		NULL,
		't',
	},
	{NULL, 0, NULL, 0}};

static int next_option;

static void print_usage(FILE *stream, int exit_code)
{
	fprintf(stream, "Usage: %s options\n", program_name);
	fprintf(stream,
			"  -h  --help           Display this usage information.\n"
			"  -v  --version        Print version.\n"
			"  -t  --timeout	timeout in seconds, 0 for indefinite wait.\n");

	exit(exit_code);
}
/***********************************************************************/

static FILE *fp = stdout;
static int timeout_provided;
static int timeout = 0;
static cyusb_handle *h1 = NULL;
static void validate_inputs(void)
{
	if ((timeout_provided) && (timeout < 0))
	{
		fprintf(stderr, "Must provide a positive value for timeout in seconds\n");
		print_usage(stdout, 1);
	}
}

static bool readerBool = false;
static void *reader(void *arg1)
{
	int r;
	constexpr size_t buffsize = 0x4000;
	unsigned char buf[buffsize];
	int transferred = 0;
	memset(buf, 0xff, buffsize);
	size_t packetCount = 0;
	while (readerBool)
	{

		r = cyusb_bulk_transfer(h1, 0x81, buf, buffsize, &transferred, 1000);
		if (r == 0)
		{
			// spdlog::info("{0} tranferred", transferred);
			for (int i = 0; i < transferred; ++i)
			{
				if (buf[i] != 0xfe)
				{
					packetCount++;
					if (packetCount > 0 && packetCount % 100000000 == 0)
					{
						spdlog::info("{0} 0xfe packet count", packetCount);
					}
				}
			}
			memset(buf, 0xff, buffsize);
			continue;
		}
		else
		{
			if (r == LIBUSB_ERROR_TIMEOUT)
			{
				spdlog::warn("There are no data to process (time out");
			}
			if (r == LIBUSB_ERROR_NO_DEVICE)
			{
				spdlog::error("Device is deiconnected break the reader loop (no device");
				break;
			}
		}
	}

	return nullptr;
}

static void *writer(void *arg2)
{
	int r, nbr;
	unsigned char buf[4096];
	unsigned int bufInt[1024];
	int transferred = 0;

	memset(buf, '\0', 4096);
	memset(bufInt, 0, 1024);
	bufInt[768] = 1;
	bufInt[769] = 0;
	bufInt[770] = 1;

	memcpy(buf, bufInt, 4096);
	int counts = 0;
	while (true)
	{
		r = cyusb_bulk_transfer(h1, 0x01, buf, nbr, &transferred, timeout * 1000);
		if (r == 0)
		{
			// memset(buf,'\0',64);
			if (counts++ == 1000)
			{
				break;
			}

			continue;
		}
		else
		{
			cyusb_error(r);
			cyusb_close();
			return NULL;
		}
	}

	printf("writing done\n");
	return NULL;
}

void callbackUSBTransferComplete(struct libusb_transfer *xfr)
{
	switch (xfr->status)
	{
	case LIBUSB_TRANSFER_COMPLETED:
	{
		// Success here, data transfered are inside
		// xfr->buffer
		// and the length is
		// xfr->actual_length
		int legnth = xfr->actual_length;
		for (int i = 0; i < legnth; ++i)
		{
			if (xfr->buffer[i] != 0xff)
			{
				printf("%02hhx", xfr->buffer[i]);
			}
		}
		break;
	}
	case LIBUSB_TRANSFER_CANCELLED:
		break;
	case LIBUSB_TRANSFER_NO_DEVICE:
		break;
	case LIBUSB_TRANSFER_TIMED_OUT:
		printf("time out\n");
		break;
	case LIBUSB_TRANSFER_ERROR:
		break;
	case LIBUSB_TRANSFER_STALL:
		break;
	case LIBUSB_TRANSFER_OVERFLOW:
		// Various type of errors here
		break;
	}
}

static size_t packetTest = 0;

void testFunc(const unsigned short* buff)
{
	++packetTest;
	if (packetTest > 0 && packetTest % 10000 == 0)
	{
		for (int i = 0; i < 148; ++i)
		{
			printf("%d ", buff[i]);
		}
		printf("\n");
	}
}

int main(int argc, char **argv)
{

	HUREL::CRUXELL::CruxellIO &cruxellIO = HUREL::CRUXELL::CruxellIO::instance();
	if (!cruxellIO.IsConnect())
	{
		spdlog::error("fx3 not connected!! exit program");
		return -1;
	}
	cruxellIO.SetByteDataAddingFunction(testFunc);
	cruxellIO.SetTestSettingValues();

	cruxellIO.Run();
	spdlog::info("press enter to terminate program");
	while (1)
	{
		int c = getchar();
		if (c == 10)
		{
			break;
		}
	}
	spdlog::info("usb connection off");
	readerBool = false;

	cruxellIO.Stop();
	return 0;
}