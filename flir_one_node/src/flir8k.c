#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <libusb.h>
#include <unistd.h>
#include <time.h>

#include <fcntl.h>
#include <math.h>

#include "jpeglib.h"
#include "Palettes.h"

// define a ramdisk folder of 20MB for jpeg images
// $ sudo mkdir /mnt/RAMDisk
// $ sudo chmod 777 /mnt/RAMDisk
// $ sudo mount -t tmpfs  -o size=20m none  /mnt/RAMDisk
#define RAMDISKPATH "/mnt/RAMDisk/"

#define VENDOR_ID 0x09cb
#define PRODUCT_ID 0x1996

static struct libusb_device_handle *devh = NULL;
int filecount = 0;
struct timeval t1, t2;
long long fps_t;

// -- buffer for EP 0x85 chunks ---------------
#define BUF85SIZE 1048576 // size got from android app
int buf85pointer = 0;
unsigned char buf85[BUF85SIZE];

void vframe(char ep[], char EP_error[], int r, int actual_length, unsigned char buf[])
{
    // error handler
    time_t now1;
    now1 = time(NULL);
    if (r < 0)
    {
        if (strcmp(EP_error, libusb_error_name(r)) != 0)
        {
            strcpy(EP_error, libusb_error_name(r));
            fprintf(stderr, "\n: %s >>>>>>>>>>>>>>>>>bulk transfer (in) %s:%i %s\n", ctime(&now1), ep, r, libusb_error_name(r));
            sleep(1);
        }
        return;
    }

    // reset buffer if the new chunk begins with magic bytes or the buffer size limit is exceeded
    unsigned char magicbyte[4] = {0xEF, 0xBE, 0x00, 0x00};

    if ((strncmp(buf, magicbyte, 4) == 0) || ((buf85pointer + actual_length) >= BUF85SIZE))
    {
        // printf(">>>>>>>>>>>begin of new frame<<<<<<<<<<<<<\n");
        buf85pointer = 0;
    }

    //printf("actual_length %d !!!!!\n", actual_length);

    memmove(buf85 + buf85pointer, buf, actual_length);
    buf85pointer = buf85pointer + actual_length;

    if ((strncmp(buf85, magicbyte, 4) != 0))
    {
        //reset buff pointer
        buf85pointer = 0;
        printf("Reset buffer because of bad Magic Byte!\n");
        return;
    }

    // a quick and dirty job for gcc
    uint32_t FrameSize = buf85[8] + (buf85[9] << 8) + (buf85[10] << 16) + (buf85[11] << 24);
    uint32_t ThermalSize = buf85[12] + (buf85[13] << 8) + (buf85[14] << 16) + (buf85[15] << 24);
    uint32_t JpgSize = buf85[16] + (buf85[17] << 8) + (buf85[18] << 16) + (buf85[19] << 24);
    uint32_t StatusSize = buf85[20] + (buf85[21] << 8) + (buf85[22] << 16) + (buf85[23] << 24);

    //printf("FrameSize= %d (+28=%d), ThermalSize %d, JPG %d, StatusSize %d, Pointer %d\n",FrameSize,FrameSize+28, ThermalSize, JpgSize,StatusSize,buf85pointer);

    if ((FrameSize + 28) > (buf85pointer))
    {
        // wait for next chunk
        return;
    }

    int i, v;
    // get a full frame, first print the status
    t1 = t2;
    gettimeofday(&t2, NULL);
    // fps as moving average over last 20 frames
    fps_t = (19 * fps_t + 10000000 / (((t2.tv_sec * 1000000) + t2.tv_usec) - ((t1.tv_sec * 1000000) + t1.tv_usec))) / 20;

    filecount++;
    printf("#%06i %lld/10 fps:", filecount, fps_t);
    for (i = 0; i < StatusSize; i++)
    {
        v = 28 + ThermalSize + JpgSize + i;
        if (buf85[v] > 31)
        {
            printf("%c", buf85[v]);
        }
    }
    printf("\n");

    buf85pointer = 0;

    unsigned short pix[160 * 120];
    int x, y;
    unsigned char *fb_proc, *fb_proc2;
    fb_proc = malloc(160 * 120);      // 8 Bit gray buffer really needs only 160 x 120
    fb_proc2 = malloc(160 * 120 * 3); // 8x8x8  Bit RGB buffer

    int min = 0x10000, max = 0;
    float rms = 0;

    // Make a unsigned short array from what comes from the thermal frame
    for (y = 0; y < 120; ++y)
    {
        for (x = 0; x < 160; ++x)
        {
            if (x < 80)
                v = buf85[2 * (y * 164 + x) + 32] + 256 * buf85[2 * (y * 164 + x) + 33];
            else
                v = buf85[2 * (y * 164 + x) + 32 + 4] + 256 * buf85[2 * (y * 164 + x) + 33 + 4];
            pix[y * 160 + x] = v; // unsigned char!!
        }
    }

    // find the max, min and RMS (not used yet) values of the array
    int maxx, maxy;
    for (i = 0; i < 160 * 120; ++i)
    {
        if (pix[i] < min)
            min = pix[i];
        if (pix[i] > max)
        {
            max = pix[i];
            maxx = i % 320;
            maxy = i / 320;
        }
        rms += pix[i] * pix[i];
    }

    // RMS used later
    rms /= 160 * 120;
    rms = sqrtf(rms);

    // scale the data in the array
    int delta = max - min;
    if (!delta)
        delta = 1;

    //  int scale = 0x10000 / (max - min); // if max = min we have divide by zero
    int scale = 0x10000 / delta;

    for (y = 0; y < 120; ++y)
    {
        for (x = 0; x < 160; ++x)
        {
            int v = (pix[y * 160 + x] - min) * scale >> 8;

            // fb_proc is the gray scale frame buffer
            fb_proc[y * 160 + x] = v; // unsigned char!!

            // fb_proc2 is an 24bit RGB buffer
            const int *colormap = colormap_ironblack;
            fb_proc2[3 * y * 160 + x * 3] = colormap[3 * v];           // unsigned char!!
            fb_proc2[(3 * y * 160 + x * 3) + 1] = colormap[3 * v + 1]; // unsigned char!!
            fb_proc2[(3 * y * 160 + x * 3) + 2] = colormap[3 * v + 2]; // unsigned char!!
        }
    }

    char filename[100];
    FILE *outfile;

    // write real image to disc
    sprintf(filename, RAMDISKPATH "real.jpg");
    if ((outfile = fopen(filename, "wb")) == NULL)
    {
        fprintf(stderr, "can't open %s\n", filename);
        exit(1);
    }
    fwrite(&buf85[28 + ThermalSize], 1, JpgSize, outfile);
    fclose(outfile);

    // write thermal frame as jpg to disc
    // see https://github.com/LuaDist/libjpeg/blob/master/example.c
    // freeze thermal image if "shutterState"="FFC"
    if (strncmp(&buf85[28 + ThermalSize + JpgSize + 17], "FFC", 3) != 0)
    {
        struct jpeg_compress_struct cinfo;
        struct jpeg_error_mgr jerr;

        int image_height = 120; /* Number of rows in image */
        int image_width = 160;  /* Number of columns in image */
        int quality = 90;

        JSAMPROW row_pointer[1]; /* pointer to JSAMPLE row[s] */
        int row_stride;          /* physical row width in image buffer */

        cinfo.err = jpeg_std_error(&jerr);
        jpeg_create_compress(&cinfo);

        sprintf(filename, RAMDISKPATH "thermal.jpg");
        if ((outfile = fopen(filename, "wb")) == NULL)
        {
            fprintf(stderr, "can't open %s\n", filename);
            exit(1);
        }
        jpeg_stdio_dest(&cinfo, outfile);

        cinfo.image_width = image_width; /* image width and height, in pixels */
        cinfo.image_height = image_height;
        cinfo.input_components = 3;     /* # of color components per pixel */
        cinfo.in_color_space = JCS_RGB; /* colorspace of input image */
        jpeg_set_defaults(&cinfo);

        jpeg_set_quality(&cinfo, quality, TRUE /* limit to baseline-JPEG values */);

        // cinfo.write_JFIF_header = FALSE;     // remove JFIF_header when needed
        jpeg_start_compress(&cinfo, TRUE);
        row_stride = image_width * 3; /* JSAMPLEs per row in image_buffer */

        while (cinfo.next_scanline < cinfo.image_height)
        {
            row_pointer[0] = &fb_proc2[cinfo.next_scanline * row_stride];
            (void)jpeg_write_scanlines(&cinfo, row_pointer, 1);
        }

        jpeg_finish_compress(&cinfo);
        fclose(outfile);

        // free memory
        jpeg_destroy_compress(&cinfo); // thermal jpg
    }

    free(fb_proc);  // thermal RAW
    free(fb_proc2); // visible jpg
}

static int find_lvr_flirusb(void)
{
    devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
    return devh ? 0 : -EIO;
}

void print_bulk_result(char ep[], char EP_error[], int r, int actual_length, unsigned char buf[])
{
    time_t now1;
    int i;

    now1 = time(NULL);
    if (r < 0)
    {
        if (strcmp(EP_error, libusb_error_name(r)) != 0)
        {
            strcpy(EP_error, libusb_error_name(r));
            fprintf(stderr, "\n: %s >>>>>>>>>>>>>>>>>bulk transfer (in) %s:%i %s\n", ctime(&now1), ep, r, libusb_error_name(r));
            sleep(1);
        }
        //return 1;
    }
    else
    {
        printf("\n: %s bulk read EP %s, actual length %d\nHEX:\n", ctime(&now1), ep, actual_length);
        // write frame to file
        /*
            char filename[100];
            sprintf(filename, "EP%s#%05i.bin",ep,filecount);
            filecount++;
            FILE *file = fopen(filename, "wb");
            fwrite(buf, 1, actual_length, file);
            fclose(file);
  */
        // hex print of first byte
        for (i = 0; i < (((200) < (actual_length)) ? (200) : (actual_length)); i++)
        {
            printf(" %02x", buf[i]);
        }

        printf("\nSTRING:\n");
        for (i = 0; i < (((200) < (actual_length)) ? (200) : (actual_length)); i++)
        {
            if (buf[i] > 31)
            {
                printf("%c", buf[i]);
            }
        }
        printf("\n");
    }
}

int EPloop(void)
{
    int i, r = 1;
    r = libusb_init(NULL);
    if (r < 0)
    {
        fprintf(stderr, "failed to initialise libusb\n");
        exit(1);
    }

    r = find_lvr_flirusb();
    if (r < 0)
    {
        fprintf(stderr, "Could not find/open device\n");
        goto out;
    }
    printf("Successfully find the Flir One G2 device\n");

    r = libusb_set_configuration(devh, 3);
    if (r < 0)
    {
        fprintf(stderr, "libusb_set_configuration error %d\n", r);
        goto out;
    }
    printf("Successfully set usb configuration 3\n");

    // Claiming of interfaces is a purely logical operation;
    // it does not cause any requests to be sent over the bus.
    r = libusb_claim_interface(devh, 0);
    if (r < 0)
    {
        fprintf(stderr, "libusb_claim_interface 0 error %d\n", r);
        goto out;
    }
    r = libusb_claim_interface(devh, 1);
    if (r < 0)
    {
        fprintf(stderr, "libusb_claim_interface 1 error %d\n", r);
        goto out;
    }
    r = libusb_claim_interface(devh, 2);
    if (r < 0)
    {
        fprintf(stderr, "libusb_claim_interface 2 error %d\n", r);
        goto out;
    }
    printf("Successfully claimed interface 0,1,2\n");

    unsigned char buf[1048576];
    int actual_length;

    time_t now;
    // save last error status to avoid clutter the log
    char EP81_error[50] = "", EP83_error[50] = "", EP85_error[50] = "";
    unsigned char data[2] = {0, 0}; // only a bad dummy

    int state = 1;
    int ct = 0;

    while (1)
    {

        switch (state)
        {

        case 1:
            /* Flir config
            01 0b 01 00 01 00 00 00 c4 d5
            0 bmRequestType = 01
            1 bRequest = 0b
            2 wValue 0001 type (H) index (L)    stop=0/start=1 (Alternate Setting)
            4 wIndex 01                         interface 1/2
            5 wLength 00
            6 Data 00 00

            libusb_control_transfer (*dev_handle, bmRequestType, bRequest, wValue,  wIndex, *data, wLength, timeout)
            */

            printf("stop interface 2 FRAME\n");
            r = libusb_control_transfer(devh, 1, 0x0b, 0, 2, data, 0, 100);
            if (r < 0)
            {
                fprintf(stderr, "Control Out error %d\n", r);
                return r;
            }

            printf("stop interface 1 FILEIO\n");
            r = libusb_control_transfer(devh, 1, 0x0b, 0, 1, data, 0, 100);
            if (r < 0)
            {
                fprintf(stderr, "Control Out error %d\n", r);
                return r;
            }

            printf("\nstart interface 1 FILEIO\n");
            r = libusb_control_transfer(devh, 1, 0x0b, 1, 1, data, 0, 100);
            if (r < 0)
            {
                fprintf(stderr, "Control Out error %d\n", r);
                return r;
            }
            now = time(0); // Get the system time
            printf("\n:xx %s", ctime(&now));
            state = 2; // jump over wait stait 2
            break;

        case 2:
            printf("\nask for CameraFiles.zip on EP 0x83:\n");
            now = time(0); // Get the system time
            printf("\n: %s", ctime(&now));

            int transferred = 0;
            char my_string[128];

            //--------- write string: {"type":"openFile","data":{"mode":"r","path":"CameraFiles.zip"}}
            int length = 16;
            unsigned char my_string2[16] = {0xcc, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x41, 0x00, 0x00, 0x00, 0xF8, 0xB3, 0xF7, 0x00};
            printf("\nEP 0x02 to be sent Hexcode: %i Bytes[", length);
            int i;
            for (i = 0; i < length; i++)
            {
                printf(" %02x", my_string2[i]);
            }
            printf(" ]\n");

            r = libusb_bulk_transfer(devh, 2, my_string2, length, &transferred, 0);
            if (r == 0 && transferred == length)
            {
                printf("\nWrite successful!");
            }
            else
                printf("\nError in write! res = %d and transferred = %d\n", r, transferred);

            strcpy(my_string, "{\"type\":\"openFile\",\"data\":{\"mode\":\"r\",\"path\":\"CameraFiles.zip\"}}");

            length = strlen(my_string) + 1;
            printf("\nEP 0x02 to be sent: %s", my_string);

            // avoid error: invalid conversion from ‘char*’ to ‘unsigned char*’ [-fpermissive]
            unsigned char *my_string1 = (unsigned char *)my_string;
            //my_string1 = (unsigned char*)my_string;

            r = libusb_bulk_transfer(devh, 2, my_string1, length, &transferred, 0);
            if (r == 0 && transferred == length)
            {
                printf("\nWrite successful!");
                printf("\nSent %d bytes with string: %s\n", transferred, my_string);
            }
            else
                printf("\nError in write! res = %d and transferred = %d\n", r, transferred);

            //--------- write string: {"type":"readFile","data":{"streamIdentifier":10}}
            length = 16;
            unsigned char my_string3[16] = {0xcc, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x33, 0x00, 0x00, 0x00, 0xef, 0xdb, 0xc1, 0xc1};
            printf("\nEP 0x02 to be sent Hexcode: %i Bytes[", length);
            for (i = 0; i < length; i++)
            {
                printf(" %02x", my_string3[i]);
            }
            printf(" ]\n");

            r = libusb_bulk_transfer(devh, 2, my_string3, length, &transferred, 0);
            if (r == 0 && transferred == length)
            {
                printf("\nWrite successful!");
            }
            else
                printf("\nError in write! res = %d and transferred = %d\n", r, transferred);

            //strcpy(  my_string, "{\"type\":\"setOption\",\"data\":{\"option\":\"autoFFC\",\"value\":true}}");
            strcpy(my_string, "{\"type\":\"readFile\",\"data\":{\"streamIdentifier\":10}}");
            length = strlen(my_string) + 1;
            printf("\nEP 0x02 to be sent %i Bytes: %s", length, my_string);

            // avoid error: invalid conversion from ‘char*’ to ‘unsigned char*’ [-fpermissive]
            my_string1 = (unsigned char *)my_string;

            r = libusb_bulk_transfer(devh, 2, my_string1, length, &transferred, 0);
            if (r == 0 && transferred == length)
            {
                printf("\nWrite successful!");
                printf("\nSent %d bytes with string: %s\n", transferred, my_string);
            }
            else
                printf("\nError in write! res = %d and transferred = %d\n", r, transferred);

            // go to next state
            now = time(0); // Get the system time
            printf("\n: %s", ctime(&now));
            //sleep(1);
            state = 3;
            break;

        case 3:
            printf("\nAsk for video stream, start EP 0x85:\n");

            r = libusb_control_transfer(devh, 1, 0x0b, 1, 2, data, 2, 200);
            if (r < 0)
            {
                fprintf(stderr, "Control Out error %d\n", r);
                return r;
            };

            state = 4;
            break;

        case 4:
            // endless loop
            // poll Frame Endpoints 0x85
            // don't change timeout=100ms !!
            r = libusb_bulk_transfer(devh, 0x85, buf, sizeof(buf), &actual_length, 100);
            if (actual_length > 0)
                vframe("0x85", EP85_error, r, actual_length, buf);

            break;
        }

        // poll Endpoints 0x81, 0x83
        r = libusb_bulk_transfer(devh, 0x81, buf, sizeof(buf), &actual_length, 10);
        print_bulk_result("0x81", EP81_error, r, actual_length, buf);

        r = libusb_bulk_transfer(devh, 0x83, buf, sizeof(buf), &actual_length, 10);
        if (strcmp(libusb_error_name(r), "LIBUSB_ERROR_NO_DEVICE") == 0)
        {
            fprintf(stderr, "EP 0x83 LIBUSB_ERROR_NO_DEVICE -> reset USB\n");
            goto out;
        }

        print_bulk_result("0x83", EP83_error, r, actual_length, buf);
    }

    // never reached ;-)
    libusb_release_interface(devh, 0);

out:
    //close the device
    libusb_reset_device(devh);
    libusb_close(devh);
    libusb_exit(NULL);
    return r >= 0 ? r : -r;
}

int main(void)
{
    while (1)
    {
        EPloop();
    }
}
