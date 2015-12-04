#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>      
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <linux/input.h>
#include <linux/hidraw.h>

#include "xunit_lib.h"

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	EnableMasterMode								    *
 *  Parameter1	:											    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to enable camera master mode			    *
  **********************************************************************************************************
*/

BOOL EnableMasterMode()
{
	int ret =0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = ENABLEMASTERMODE; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	return TRUE;
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	EnableTriggerMode								    *
 *  Parameter1	:											    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to enable camera trigger mode			    *
  **********************************************************************************************************
*/

BOOL EnableTriggerMode()
{
	int ret =0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = ENABLETRIGGERMODE; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	return TRUE;
}

/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	EnableBinnedVGAMode								    *
 *  Parameter1	:	UINT8 * ( Binned VGA status )							    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to enable camera binned VGA mode		    *	
  **********************************************************************************************************
*/

BOOL EnableBinnedVGAMode(UINT8 *VGAStatus)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = ENABLE_BINNED_VGA_MODE; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if(g_in_packet_buf[0] == ENABLE_BINNED_VGA_MODE) {
				*VGAStatus = g_in_packet_buf[1];
				timeout = FALSE;
			}
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = FALSE;
			return FALSE;
		}		
	}

	return TRUE;
}


/*
  **********************************************************************************************************
 *  MODULE TYPE	:	LIBRAY API 									    *
 *  Name	:	EnableCroppedVGAMode								    *
 *  Parameter1	:	UINT8 * ( Cropped VGA status )							    *
 *  Parameter2	:											    *
 *  Returns	:	BOOL (TRUE or FALSE)								    *
 *  Description	:       sends the extension unit command to enable camera cropped VGA mode		    *	
  **********************************************************************************************************
*/

BOOL EnableCroppedVGAMode(UINT8 *VGAStatus)
{
	BOOL timeout = TRUE;
	int ret =0;
	unsigned int start, end = 0;

	//Initialize the buffer
	memset(g_out_packet_buf, 0x00, sizeof(g_out_packet_buf));

	//Set the Report Number
	g_out_packet_buf[1] = ENABLE_CROPPED_VGA_MODE; /* Report Number */

	ret = write(hid_fd, g_out_packet_buf, BUFFER_LENGTH);
	if (ret < 0) {
		perror("write");
		return FALSE;
	} else {
		printf("%s(): write() wrote %d bytes\n", __func__, ret);
	}
	start = GetTickCount();
	while(timeout) 
	{	
		/* Get a report from the device */
		ret = read(hid_fd, g_in_packet_buf, BUFFER_LENGTH);
		if (ret < 0) {
			//perror("read");
		} else {
			printf("%s(): read %d bytes:\n", __func__,ret);
			if(g_in_packet_buf[0] == ENABLE_CROPPED_VGA_MODE) {
				*VGAStatus = g_in_packet_buf[1];
				timeout = FALSE;
			}
	 	}
		end = GetTickCount();
		if(end - start > TIMEOUT)
		{
			printf("%s(): Timeout occurred\n", __func__);
			timeout = FALSE;
			return FALSE;
		}		
	}

	return TRUE;
}

BOOL SetCroppedVGAMode()
{
	BOOL ret = FALSE;
	UINT8 cropped_vga_status = 0;
	ret = EnableCroppedVGAMode(&cropped_vga_status);
	if(ret == FALSE)
	{
		printf("Unable to switch to cropped VGA Mode\n");
		return;
	}
	else
	{
		switch(cropped_vga_status)
		{
			case 1 : 
				printf("Cropped VGA mode set successfully\n");
				break;
			case 2 :
				printf("The current resolution is not 640x480, please switch to 640x480 before using the Cropping and Binning modes\n");
				break;
			case 3 :
				printf("Device is already in Cropped VGA mode\n");
				break;
			case 4 :
				printf("Failed to set Cropped VGA mode\n");
				break;
			default :
				printf("Failed to set Cropped VGA mode - unknown basis\n ");
				
		}
	}
	return ret;
}

BOOL SetBinnedVGAMode()
{
	BOOL ret = FALSE;
	UINT8 binned_vga_status = 0;
	ret = EnableBinnedVGAMode(&binned_vga_status);
	if(ret == FALSE)
	{
		printf("Unable to switch to binned VGA Mode\n");
		return;
	}
	else
	{
		switch(binned_vga_status)
		{
			case 1 : 
				printf("Binned VGA mode set successfully\n");
				break;
			case 2 :
				printf("The current resolution is not 640x480, please switch to 640x480 before using the Cropping and Binning modes\n");
				break;
			case 3 :
				printf("Device is already in Binned VGA mode\n");
				break;
			case 4 :
				printf("Failed to set Binned VGA mode\n");
				break;
			default :
			  printf("Failed to set Binned VGA mode - unknown basis\n");
		}
	}
	return ret;
}
