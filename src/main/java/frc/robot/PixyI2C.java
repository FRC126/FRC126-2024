/**********************************
	   _      ___      ____
	 /' \   /'___`\   /'___\
	/\_, \ /\_\ /\ \ /\ \__/
	\/_/\ \\/_/// /__\ \  _``\
	   \ \ \  // /_\ \\ \ \L\ \
	    \ \_\/\______/ \ \____/
		 \/_/\/_____/   \/___/

    Team 126 2022 Code       
	Go get em gaels!

***********************************/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public class PixyI2C {
	String name;
	I2C pixy;
	PixyException pExc;

	/************************************************************************
	 ************************************************************************/

	public PixyI2C(String id, I2C argPixy) {
		pixy = argPixy;
		name = "Pixy_" + id;
	}

	/************************************************************************
	 ************************************************************************/

	// Convert the raw bytes in integers with endian conversion
	public int cvt(byte upper, byte lower) {
		return (((int) upper & 0xff) << 8) | ((int) lower & 0xff);
	}

	/************************************************************************
	 ************************************************************************/

	public byte getByte() {
		byte[] rawData = new byte[1];

		try {
			pixy.readOnly(rawData, 1);
		} catch (RuntimeException e) {
			//System.out.println("getByte RuntimeException " + e.toString());
			return 0;
		}
		if (rawData.length < (1)) {
			//System.out.println("getByte rawData.length " + rawData.length);
			return 0;
		}
		return (rawData[0]);
	}

	/************************************************************************
	 ************************************************************************/

    private int readResponse() {
		byte b1, b2;
		byte[] rawData = new byte[256];

		b1 = getByte();
		b2 = getByte();

		// Verify Packet Signature 
		if ( b2 != (byte)0xc1 || b1 != (byte)0xaf) {
			System.out.println(String.format("Did not find start of frame, b1: 0x%02X b2 0x%02X", b1,b2));
			return -1;
		}	
		if (getByte() != (byte)1) {
			System.out.println("Did not find good packet type");
			return -1;
		}		

		int len = (int)getByte();
		// Read the rest of the response
		if ( len > 0 ) {
			pixy.readOnly(rawData,len);
		}
		
		return 0;
	}

	/************************************************************************
	 ************************************************************************/

	public int setLED(int red, int green, int blue) throws PixyException {
		/*
		setLED(r, g, b)

		Request:
			Byte	Description	Value(s)
			0 - 1	16-bit sync	174, 193 (0xc1ae)
			2	Type of packet	20
			3	Length of payload	3
			4	r - Red value	0 - 255
			5	g - Green value	0 - 255
			6	b - Blue value	0 - 255

		Response:
			Byte	Description	Value(s)
			0 - 1	16-bit sync	175, 193 (0xc1af)
			2	Type of packet	1
			3	Length of payload	4
			4 - 5	16-bit checksum	sum of payload bytes
			6 - 9	32-bit result/acknowledge	result value
		*/	

		byte[] rawData = new byte[256];

		// Sync Value
		rawData[1]=(byte)0xc1;
		rawData[0]=(byte)0xae;
		
		// Packet Type
		rawData[2]=(byte)20;

		// Packet Length 
		rawData[3]=(byte)3;

		// Set Color Values
		rawData[4]=(byte)red;
		rawData[5]=(byte)green;
		rawData[6]=(byte)blue;

		if (pixy.writeBulk(rawData, 7)) {
			System.out.println("Error sending setLED message");
			return -1;
		}	

		return readResponse();
	}	

	/************************************************************************
	 ************************************************************************/

	public int setServo(int leftRight, int upDown) throws PixyException {
		/*
		setServos(s0, s1)

		Request:
			Byte	Description	Value(s)
			0 - 1	16-bit sync	174, 193 (0xc1ae)
			2	Type of packet	18
			3	Length of payload	4
			4 - 5	16-bit s0 - pan servo value	0 - 511
			6 - 7	16-bit s1 - tilt servo value	0 - 511

		Response:
			Byte	Description	Value(s)
			0 - 1	16-bit sync	175, 193 (0xc1af)
			2	Type of packet	1
			3	Length of payload	4
			4 - 5	16-bit checksum	sum of payload bytes
			6 - 9	32-bit result/acknowledge	result value
		*/

		byte[] rawData = new byte[256];
	
		if (leftRight < 0 || leftRight > 511) {
			System.out.println("Invalid leftRight value pased to setServo");
            return -1;			
		}

		if (upDown < 0 || upDown > 511) {
			System.out.println("Invalid upDown value pased to setServo");
            return -1;			
		}

		// Sync Byte 0x1ae
		rawData[0]=(byte)0xae;	
		rawData[1]=(byte)0xc1;

		// Packet Type
		rawData[2]=(byte)18;

		// Packet Length 
		rawData[3]=(byte)4;
	 
		// set leftRight Postion 0 - 511
		rawData[4] = (byte)(leftRight & 255);
		rawData[5] = (byte)(leftRight >> 8); 

		// set upDown Postion 0 - 511
		rawData[6] = (byte)(upDown & 255);
		rawData[7] = (byte)(upDown >> 8); 
			
		if (pixy.writeBulk(rawData, 8)) {
			System.out.println("Error sending setServo message");
			return -1;
		}
		
		return readResponse();
	}	

	/************************************************************************
	 ************************************************************************/

	public int setLamp(boolean upperOn, boolean lowerOn) throws PixyException {
		/*
		setLamp(upper, lower)

		Request:
			Byte	Description	Value(s)
			0 - 1	16-bit sync	174, 193 (0xc1ae)
			2	Type of packet	22
			3	Length of payload	2
			4	Upper - turn on the two white LEDs along Pixy2 top edge	0 (off) or 1 (on)
			5	Lower - turn on all channels of lower RGB LED	0 (off) or 1 (on)

		Response:
			Byte	Description	Value(s)
			0 - 1	16-bit sync	175, 193 (0xc1af)
			2	Type of packet	1
			3	Length of payload	4
			4 - 5	16-bit checksum	sum of payload bytes
			6 - 9	32-bit result/acknowledge	result value
		*/

		byte[] rawData = new byte[256];
	
		// Sync Byte 0x1ae
		rawData[0]=(byte)0xae;	
		rawData[1]=(byte)0xc1;

		// Packet Type
		rawData[2]=(byte)22;

		// Packet Length 
		rawData[3]=(byte)2;
	 
		// set UpperLamp
		if (upperOn) {
			rawData[4] = (byte)1;
		} else {
			rawData[4] = (byte)0;
		}	

		// set UpperLamp
		if (lowerOn) {
			rawData[5] = (byte)1;
		} else {
			rawData[5] = (byte)0;
		}	
	
		if (pixy.writeBulk(rawData, 6)) {
			System.out.println("Error sending setLamp message");
			return -1;
		}
		
		return readResponse();
	}	

	/************************************************************************
	 ************************************************************************/

	public int setCameraBrightness(int brightness) throws PixyException {
		/*
		setCameraBrightness(brightness)

		Request:
			Byte	Description	Value(s)
			0 - 1	Sync	174, 193 (0xc1ae)
			2	Type of packet	16
			3	Length of payload	1
			4	Brightness	0 - 255

		Response:
			Byte	Description	Value(s)
			0 - 1	16-bit sync	175, 193 (0xc1af)
			2	Type of packet	1
			3	Length of payload	4
			4 - 5	16-bit checksum	sum of payload bytes
			6 - 9	32-bit result	result value
		*/

		if (brightness < 0 || brightness > 255) {
			System.out.println("Invalid brightness value pased to setCameraBrightness");
            return -1;			
		}

		byte[] rawData = new byte[256];
	
		// Sync Byte 0x1ae
		rawData[0]=(byte)0xae;	
		rawData[1]=(byte)0xc1;

		// Packet Type
		rawData[2]=(byte)16;

		// Packet Length 
		rawData[3]=(byte)1;
	 
		// set Brightness
		rawData[4] = (byte)brightness;

		if (pixy.writeBulk(rawData, 5)) {
			System.out.println("Error sending setCameraBrightness message");
			return -1;
		}
		
		return readResponse();
	}		

	/************************************************************************
	 ************************************************************************/

	public int getBlocks(int objectId, int maxBlocks, PixyPacket[] dataPackets) throws PixyException {
		/*
		getBlocks(sigmap, maxBlocks)
		Request:
			Bit	Description	Value(s)
			0 - 1	16-bit sync	174, 193 (0xc1ae)
			2	Type of packet	32
			3	Length of payload	2
			4	Sigmap - indicate which signatures to receive data from	0 (none) - 255 (all)
			5	Maximum blocks to return	0 (none) - 255 (all blocks)

		Response:
			Byte	Description	Value(s)
			0 - 1	16-bit sync	175, 193 (0xc1af)
			2	Type of packet	33
			3	Length of payload	14
			4 - 5	16-bit checksum	sum of payload bytes
			6 - 7	16-bit signature / Color code number	0 - 255
			8 - 9	16-bit X (center) of block in pixels	0 - 315
			10 - 11	16-bit Y (center) of block in pixels	0 - 207
			12 - 13	16-bit Width of block in pixels	0 - 316
			14 - 15	16-bit Height of block in pixels	0 - 208
			16 - 17	16-bit Angle of color-code in degrees	-180 - 180 (0 if not a color code)
			18	Tracking index of block (see API for more info)	0 - 255
			19	Age - number of frames this block has been tracked	0 - 255 (stops incrementing at 255)
		*/

		// Validate the objectID
		if (objectId < 0 || objectId > 16) {
			System.out.println("Invalid objectId value pased to getBlocks");
            return -1;			
		}

		// Validate maxBlocks, right now we only support 1
		if (maxBlocks <= 0 || maxBlocks > 1) {
			System.out.println("Invalid maxBlocks value pased to getBlocks");
            return -1;			
		}

		byte b1, b2;
		byte[] rawData = new byte[256];
	
		// Sync Byte 0x1ae
		rawData[0]=(byte)0xae;	
		rawData[1]=(byte)0xc1;

		// Packet Type
		rawData[2]=(byte)32;

		// Packet Length 
		rawData[3]=(byte)2;
	 
		// set sigMap
		rawData[4] = (byte)objectId;

		// set maxBlocks
		rawData[5] = (byte)maxBlocks;

		if (pixy.writeBulk(rawData, 6)) {
			System.out.println("Error sending getBlocks message");
			return -1;
		}

		dataPackets[objectId].initPacket();
		
		// Read the response packet
		b1 = getByte();
		b2 = getByte();

		// Verify Packet Signature 
		if ( b2 != (byte)0xc1 || b1 != (byte)0xaf) {
			System.out.println(String.format("Did not find start of frame, b1: 0x%02X b2 0x%02X", b1,b2));
			return -1;
		}	

		// Get the packet type and verify it's the proper response
		b1 = getByte();
		if (b1 != (byte)33) {
			System.out.println("Did not find good packet type " + (int)b1);
			return -1;
		}		

		// Get the length of the respone.  Will be 0 if no object of the specified 
		// type is being tracked by the camera
		int len = (int)getByte();
		
		if (len == 14) {
			// Read the rest of the response
    		pixy.readOnly(rawData,14);

			// int checksum = cvt(rawData[1], rawData[0]);
			// int colorCodeAngle = cvt(rawData[13], rawData[12]);
			// int trackingIndex = (int)rawData[14];
			// int trackingAge = (int)rawData[15];

			dataPackets[objectId].X = cvt(rawData[5], rawData[4]);
			dataPackets[objectId].Y = cvt(rawData[7], rawData[6]);
			dataPackets[objectId].Height = cvt(rawData[9], rawData[8]);
			dataPackets[objectId].Width = cvt(rawData[11], rawData[10]);
			dataPackets[objectId].Signature = cvt(rawData[3], rawData[2]);
			dataPackets[objectId].isValid = true;

			return 1;
		}
		return -1;
	}		
}