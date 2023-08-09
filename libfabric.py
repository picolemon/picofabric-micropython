"""
Provides an interface to the PicoFabric FPGA over SPI and accessing board features.

Example usage:

- Upload a bit stream to the Pico flash storage using Thonny or Pico-W-go file sync.

- Upload the bit stream to the FPGA device.
import libfabric
libfabric.fpga_program_device( filename="bitstream.bit", config=FPGAConfig() ):

"""
import machine, time


class FPGABoardId:
    """
        Supported board Ids.
    """
    BOARD_FABRIC12k = 0x1


class FPGADeviceIds:
    """
        Device ID's reported by FPGA chip.
    """
    DEVID_LFE5U_12 = 0x21111043


class FPGACommands:
    """
        ECP5 FPGA SPI commands
    """
    CMD_READ_ID  = 0xE0
    CMD_USERCODE  = 0xC0
    CMD_ISC_ENABLE  = 0xC6
    CMD_LSC_BITSTREAM_BURST  = 0x7A
    CMD_LSC_CHECK_BUSY  =  0xF0
    CMD_ISC_DISABLE  = 0x26


class FPGAConfig:
    """
        FPGA SPI config
    """
    def __init__( s, boardId=FPGABoardId.BOARD_FABRIC12k,
                  csn=13, sck=10, mosi=11, miso=12,
                  programn=15, spiId=1 ):
        """
            :param int boardId: Board to program. see FPGABoardId.
            :param int csn: FPGA chip select gpio pin number.
            :param int sck: FPGA clock gpio pin number.
            :param int mosi: FPGA MOSI gpio pin number.
            :param int miso: FPGA MISO gpio pin number.
            :param int programn: FPGA programn reset pin number.        
            :param int spiId: FPGA spi device id.            
        """

        # create IO pins
        s.csn = machine.Pin(csn, machine.Pin.OUT)
        s.sck = machine.Pin(sck, machine.Pin.OUT)
        s.mosi = machine.Pin(mosi, machine.Pin.OUT)
        s.miso = machine.Pin(miso, machine.Pin.IN)
        s.programn = machine.Pin(programn, machine.Pin.OUT)        


        # create SPI instance
        s.spi = machine.SPI(spiId,
                  baudrate=12000000,    # 48Mhz / 4 ( slow for edge sampler as runs on single clock domain )
                  polarity=1,
                  phase=1,
                  bits=8,
                  firstbit=machine.SPI.MSB,
                  sck=s.sck,
                  mosi=s.mosi,
                  miso=s.miso)



class DACConfig:
    """
        DAC config.
    """
    def __init__( s, csn=5, sck=6, mosi=7, sysClock=12000000, sampleRate=22058, spiId=1, vol=0.5, balance=0,
                  i2s_sck=2, i2s_ws=3, i2s_sd=4, i2s_id=0 ):
        """
            DAC SPI routing and sample rate config.
            
            :param int csn: SPI chip select gpio pin number, routed through the fabric to DAC.
            :param int sck: SPI clock gpio pin number, routed through the fabric to DAC.
            :param int mosi: SPI MOSI gpio pin number, routed through the fabric to DAC.
            :param int sysClock: (Reserved for future use) Specify DAC System Clock. Will raise error on un-supported values.
            :param int sampleRate: (Reserved for future use) Specify DAC Sample rate. Will raise error on un-supported values.
            :param int spiId: SPI device index.
            :param float vol: Volume from 0 - 1.
            :param float balance: Balance from -1 - +1.
            :param int i2s_sck: I2S Clock pin ( BCK )
            :param int i2s_ws: I2S Word clock pin ( LCK )
            :param int i2s_sd: I2S Word data pin ( DIN )
            :param int i2s_id: I2S device id.            
        """
        s.csn =  machine.Pin(csn, machine.Pin.OUT)
        s.sck =  machine.Pin(sck, machine.Pin.OUT)
        s.mosi =  machine.Pin(mosi, machine.Pin.OUT)
        s.sysClock = sysClock
        s.sampleRate = sampleRate
        s.vol = vol
        s.balance = balance

        # create SPI instance
        s.spi = machine.SPI(0,
                        baudrate=1000000,
                        polarity=1,
                        phase=1,
                        bits=8,
                        firstbit=machine.SPI.MSB,
                        sck=s.sck,
                        mosi=s.mosi,
                        miso=None)

        # I2S pins
        # NOTE: not defining I2S instance as is very specific to playback implementation.
        s.i2s_id = i2s_id
        s.i2s_sck = i2s_sck
        s.i2s_ws = i2s_ws
        s.i2s_sd = i2s_sd
                

class FabricBusConfig:
    """
        Fabric IO bus SPI config
    """
    def __init__( s, csn=3, sck=10, mosi=11, miso=12, spiId=1, busWidth=1 ):
        """
            :param int csn: Bus chip select gpio pin number.
            :param int sck: Bus clock gpio pin number.
            :param int mosi: Bus MOSI gpio pin number.
            :param int miso: Bus MISO gpio pin number.            
            :param int spiId: Bus spi device id.
            :param int busWidth: Bus width (1 bit max)
        """

        # create IO pins
        s.csn = machine.Pin(csn, machine.Pin.OUT)
        s.sck = machine.Pin(sck, machine.Pin.OUT)
        s.mosi = machine.Pin(mosi, machine.Pin.OUT)
        s.miso = machine.Pin(miso, machine.Pin.IN)

        # create SPI instance
        s.spi = machine.SPI(spiId,
                  baudrate=1000000,
                  polarity=1,
                  phase=1,
                  bits=8,
                  firstbit=machine.SPI.MSB,
                  sck=s.sck,
                  mosi=s.mosi,
                  miso=s.miso)


def fpga_read( config, cmd, sz ):
    """
        Read command from FPGA
    """
    dataout = [
        cmd
    ]
    config.csn.value(0)
    config.spi.write(bytearray(dataout));
    result = config.spi.read(sz);
    config.csn.value(1)
    return result
    
    
def fpga_read_id( config ):
    """
        Read FPGA device ID.
    """
    buf = fpga_read( config, FPGACommands.CMD_READ_ID, 8 );
    return (buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | (buf[6] << 0);


def fpga_poll_busy( config ):
    """
        Read FPGA busy status register.
    """
    readSz = 1;
    buf = fpga_read(config, FPGACommands.CMD_LSC_CHECK_BUSY, 3+readSz);	
    return buf[3];


def fpga_isc_enable( config ):
    """
        FPGA enables ISC mode.
    """
    fpga_read(config, FPGACommands.CMD_ISC_ENABLE, 3); # cmd + dummy ( Class C cmd ) 


def fpga_isc_disable( config ):
    """
        FPGA disables ISC mode.
    """    
    fpga_read(config, FPGACommands.CMD_ISC_DISABLE, 3); # cmd + dummy ( Class C cmd ) 


def fpga_write_bitstream_fp( config, fp, debugLevel=0 ):
    """
        Internal write fpga bit stream to FPGA, the device must be in
        ISC mode for this operation to succeed.
    """
    config.csn.value(0)
    burstCmd = [FPGACommands.CMD_LSC_BITSTREAM_BURST, 0, 0, 0 ]
    config.spi.write( bytearray(burstCmd)); # write cmd    
    totalBytes = 0
    while True:
        data = fp.read(1024)
        if not data:
            break
        if debugLevel > 1:
            print("[PROGRESS] write block", len(data), totalBytes )
        config.spi.write(data); # write bitStream payload block   
        totalBytes += len(data)

    config.csn.value(1)
    
    time.sleep(0.1);
    if debugLevel > 1:
        print("[SUCCESS] BitStream written")
    return True


def fpga_program_device_fp( config, fp, debugLevel=0 ):
    """
        Program file stream to the FPGA.
    """
    config.csn.value(1);
    config.sck.value(0);
    config.mosi.value(0);    
    config.programn.value(0);
    time.sleep(0.1)
    
    # FPGA_PROGRAMN_PIN enable
    config.programn.value(1)
    time.sleep(0.1)
    
    # toggle program pin
    config.programn.value(0);
    time.sleep(0.1)
    config.programn.value(1);
    time.sleep(0.1)    
    
    deviceId = fpga_read_id( config );
    if debugLevel > 1:
        print("[DEBUG] Read deviceId: %X" % deviceId);
        
    if(deviceId != FPGADeviceIds.DEVID_LFE5U_12):	
        raise Exception("Program failed, invalid device Id: %s" % str(deviceId));
    
    # Ensure device not busy
    isBusy = fpga_poll_busy( config );
    if debugLevel > 1:
        print("[DEBUG] fpga_program_device_fp isBusy: %s" % str(isBusy)); 
        
    if(isBusy):
        if debugLevel > 0:
            print("[ERROR] fpga_program_device_fp failed, device busy.");
        return False

    # Enable isc mode
    fpga_isc_enable( config );

    # write bit stream
    fpga_write_bitstream_fp( config, fp, debugLevel=debugLevel );

    # Disable config mode
    fpga_isc_disable(  config );

    # Check complete
    isBusy = fpga_poll_busy(  config );
    if(isBusy):
        if debugLevel > 0:
            print("[ERROR] fpga_program_device_fp failed, device busy.");
        return False

    return True


def fpga_program_device( filename, config=FPGAConfig(), debugLevel=0 ):
    """
        Program the FPGA with a given file on the devices flash filesystem storage.

        :param str filename: FPGA Bit Stream filename on the local file system.
        :param FPGAConfig config: FPGA configuration object for spi pin mappings.
        :param int debugLevel: Debug log level, trace debug events to console.
    """

    assert config, "Missing config"

    # Program using file reference ( if exists )
    return fpga_program_device_fp( config, open(filename, "rb"), debugLevel=debugLevel )


def dac_write_register( config, reg, val ):
    """
        Write dac register.
    """
    config.csn.value(0) 
    config.spi.write( bytearray([reg,val])); # write cmd
    config.csn.value(1)


def dac_init( config=DACConfig() ):
    """
        Initialise the onboard audio Dac over an SPI bridge. The audio DAC spi interface is
        routed from the Pico through the fabric to the DAC. This depends on the bitstream mapping
        to gpio and requires a little vhdl route and do the expected clocks.
        
        The (pcm1774) DAC clock defaults to 12Mhz and a sample rate of 22.058 Khz as defined by
        the RTL logic.
    """

    # check for supported config values
    assert config, "Missing config"
    assert config.sysClock == 12000000, "Invalid DAC System clock"
    assert config.sampleRate == 22058, "Invalid DAC sample rate"
    # see pcm1774 datasheet for register reference

    # Headphone vol calc + balance shift   
    vol = min(1, max(0, config.vol));
    balance = min(1, max(-1, config.balance))

    L, R = 0,0    
    if balance < 0:
        L = vol
        R = vol - (-balance*vol)
    elif balance > 0:
        L = vol - (balance*vol)
        R = vol
    else:
        L, R = vol, vol

    # convert to 0x3F range
    L =  min(0x3F, max(0,  int(L*0x3F) ) )
    R =  min(0x3F, max(0, int(R*0x3F) ) )

    dac_write_register(config, PCM1774Registers.HPA_LCH_REG, L); 
    dac_write_register(config, PCM1774Registers.HPA_RCH_REG, R); 
    dac_write_register(config, PCM1774Registers.ALT_LCH_REG, 0x3F); # Max
    dac_write_register(config, PCM1774Registers.ALT_RCH_REG, 0x3F);  # Max      
        
    #  Non burst 12mhz sysClock, 22.058 khz sample rate eg. SCK/544.
    dac_write_register(config, PCM1774Registers.AIFACE_REG, 0x00); # Default i2s
    dac_write_register(config, PCM1774Registers.IFACESEL_REG, 0x01); # 16 bit Slave mode
    dac_write_register(config, PCM1774Registers.SYS55_REG, 0x4);  
    dac_write_register(config, PCM1774Registers.SYS56_REG, 0x20); 
        
    dac_write_register(config, PCM1774Registers.PMXR_EN_REG, 0x03); # Enable mixer
    dac_write_register(config, PCM1774Registers.MXRSW_REG, 0x11); # Mixer switch
    dac_write_register(config, PCM1774Registers.HPA_PWR_REG, 0xEC);
    dac_write_register(config, PCM1774Registers.AOUT_REG, 0x01); # power up vcom	        


class PCM1774Registers:
    """
    PCM1774 DAC registers.
    """
    HPA_LCH_REG = 0x40      # Volume for HPA (L-ch)
    HPA_RCH_REG = 0x41      # Volume for HPA (R-ch)
    ALT_LCH_REG = 0x44      # DAC digital attenuation and soft mute (L-ch)
    ALT_RCH_REG = 0x45      # DAC digital attenuation and soft mute (R-ch)
    AIFACE_REG = 0x46       # DAC over sampling, de-emphasis, audio interface, DGC
    PMXR_EN_REG = 0x48      # Analog mixer power up/down
    IFACESEL_REG = 0x54     # Master mode
    SYS55_REG = 0x55        # System reset, sampling rate control, data swap
    SYS56_REG = 0x56        # BCK configuration, sampling rate control, zero-cross 
    MXRSW_REG = 0x58        # Analog mixing switch
    HPA_PWR_REG = 0x49      # DAC and HPA power up/down
    AOUT_REG = 0x4A         # Analog output configuration select
