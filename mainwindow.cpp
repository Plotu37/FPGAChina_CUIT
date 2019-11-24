#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <unistd.h>
#include <opencv2/features2d/features2d.hpp>
#include <QTime>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->close_video_pushButton->setDisabled(true);
    ui->Snapshot->setDisabled(true);
    ui->Stitch_pushButton->setDisabled(true);
    ui->orb_pushButton->setDisabled(true);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::image_show(QLabel *label, Mat image)
{
    Mat rgb;
    QImage img;

    if(image.channels()==3)
    {
        cvtColor(image,rgb,CV_BGR2RGB);
        img =QImage((const unsigned char*)(rgb.data),
                    rgb.cols,rgb.rows,
                    rgb.cols*rgb.channels(),
                    QImage::Format_RGB888);
    }
    else if(image.channels()==1)
    {
        img =QImage((const unsigned char*)(image.data),
                    image.cols,image.rows,
                    image.cols*image.channels(),
                    QImage::Format_Indexed8);
    }
    else
    {
        img =QImage((const unsigned char*)(image.data),
                    image.cols,image.rows,
                    image.cols*image.channels(),
                    QImage::Format_RGB888);
    }

    label->setPixmap(QPixmap::fromImage(img));
    label->resize(label->pixmap()->size());
}


void MainWindow::sccb_senddata(unsigned short addr,unsigned char value)
{
    unsigned char buf[3];
    buf[0] = (addr >> 8);
    buf[1] = addr;
    buf[2] = value;
    cam0_fd = ::open(CAMERA_ov5640_0_DEVICE, O_RDWR);
    if (cam0_fd < 0) {
            printf("Open %s failed\n", CAMERA_ov5640_0_DEVICE);
           // return -1;
    }
    write(cam0_fd,&buf,3);
}
void MainWindow::sccb_senddata1(unsigned short addr,unsigned char value)
{
    unsigned char buf[3];
    buf[0] = (addr >> 8);
    buf[1] = addr;
    buf[2] = value;
    cam1_fd = ::open(CAMERA_ov5640_1_DEVICE, O_RDWR);
    if (cam1_fd < 0) {
            printf("Open %s failed\n", CAMERA_ov5640_1_DEVICE);
           // return -1;
    }
    write(cam1_fd,&buf,3);
}

void MainWindow::ov5640_init_mode_1(void)
{
    sccb_senddata(0x3103,0x11);// system clock from pad,0x bit[1]
    sccb_senddata(0x3008,0x82);// software reset,0x bit[7]// delay 5ms
    sccb_senddata(0x3008,0x42);// software power down,0x bit[6]
    sccb_senddata(0x3103,0x03);// system clock from PLL,0x bit[1]
    sccb_senddata(0x3017,0xff);// FREX,0x Vsync,0x HREF,0x PCLK,0x D[9:6] output enable
    sccb_senddata(0x3018,0xff);// D[5:0],0x GPIO[1:0] output enable
    sccb_senddata(0x3034,0x1A);// MIPI 10-bit
    sccb_senddata(0x3037,0x13);// PLL root divider,0x bit[4],0x PLL pre-divider,0x bit[3:0]
    sccb_senddata(0x3108,0x01);// PCLK root divider,0x bit[5:4],0x SCLK2x root divider,0x bit[3:2] // SCLK root divider,0x bit[1:0]
    sccb_senddata(0x3630,0x36);
    sccb_senddata(0x3631,0x0e);
    sccb_senddata(0x3632,0xe2);
    sccb_senddata(0x3633,0x12);
    sccb_senddata(0x3621,0xe0);
    sccb_senddata(0x3704,0xa0);
    sccb_senddata(0x3703,0x5a);
    sccb_senddata(0x3715,0x78);
    sccb_senddata(0x3717,0x01);
    sccb_senddata(0x370b,0x60);
    sccb_senddata(0x3705,0x1a);
    sccb_senddata(0x3905,0x02);
    sccb_senddata(0x3906,0x10);
    sccb_senddata(0x3901,0x0a);
    sccb_senddata(0x3731,0x12);
    sccb_senddata(0x3600,0x08);// VCM control
    sccb_senddata(0x3601,0x33);// VCM control
    sccb_senddata(0x302d,0x60);// system control
    sccb_senddata(0x3620,0x52);
    sccb_senddata(0x371b,0x20);
    sccb_senddata(0x471c,0x50);
    sccb_senddata(0x3a13,0x43);// pre-gain = 1.047x
    sccb_senddata(0x3a18,0x00);// gain ceiling
    sccb_senddata(0x3a19,0xf8);// gain ceiling = 15.5x
    sccb_senddata(0x3635,0x13);
    sccb_senddata(0x3636,0x03);
    sccb_senddata(0x3634,0x40);
    sccb_senddata(0x3622,0x01); // 50/60Hz detection     50/60Hz
    sccb_senddata(0x3c01,0x34);// Band auto,0x bit[7]
    sccb_senddata(0x3c04,0x28);// threshold low sum
    sccb_senddata(0x3c05,0x98);// threshold high sum
    sccb_senddata(0x3c06,0x00);// light meter 1 threshold[15:8]
    sccb_senddata(0x3c07,0x08);// light meter 1 threshold[7:0]
    sccb_senddata(0x3c08,0x00);// light meter 2 threshold[15:8]
    sccb_senddata(0x3c09,0x1c);// light meter 2 threshold[7:0]
    sccb_senddata(0x3c0a,0x9c);// sample number[15:8]
    sccb_senddata(0x3c0b,0x40);// sample number[7:0]
    sccb_senddata(0x3810,0x00);// Timing Hoffset[11:8]
    sccb_senddata(0x3811,0x10);// Timing Hoffset[7:0]
    sccb_senddata(0x3812,0x00);// Timing Voffset[10:8]
    sccb_senddata(0x3708,0x64);
    sccb_senddata(0x4001,0x02);// BLC start from line 2
    sccb_senddata(0x4005,0x1a);// BLC always update
    sccb_senddata(0x3000,0x00);// enable blocks
    sccb_senddata(0x3004,0xff);// enable clocks
    sccb_senddata(0x300e,0x58);// MIPI power down,0x DVP enable
    sccb_senddata(0x302e,0x00);
    sccb_senddata(0x4300,0x61);// RGB565
    sccb_senddata(0x501f,0x01);// ISP RGB
    sccb_senddata(0x440e,0x00);
    sccb_senddata(0x5000,0xa7); // Lenc on,0x raw gamma on,0x BPC on,0x WPC on,0x CIP on // AEC target
    sccb_senddata(0x3a0f,0x30);// stable range in high
    sccb_senddata(0x3a10,0x28);// stable range in low
    sccb_senddata(0x3a1b,0x30);// stable range out high
    sccb_senddata(0x3a1e,0x26);// stable range out low
    sccb_senddata(0x3a11,0x60);// fast zone high
    sccb_senddata(0x3a1f,0x14);// fast zone low// Lens correction for
    sccb_senddata(0x5800,0x23);
    sccb_senddata(0x5801,0x14);
    sccb_senddata(0x5802,0x0f);
    sccb_senddata(0x5803,0x0f);
    sccb_senddata(0x5804,0x12);
    sccb_senddata(0x5805,0x26);
    sccb_senddata(0x5806,0x0c);
    sccb_senddata(0x5807,0x08);
    sccb_senddata(0x5808,0x05);
    sccb_senddata(0x5809,0x05);
    sccb_senddata(0x580a,0x08);
    sccb_senddata(0x580b,0x0d);
    sccb_senddata(0x580c,0x08);
    sccb_senddata(0x580d,0x03);
    sccb_senddata(0x580e,0x00);
    sccb_senddata(0x580f,0x00);
    sccb_senddata(0x5810,0x03);
    sccb_senddata(0x5811,0x09);
    sccb_senddata(0x5812,0x07);
    sccb_senddata(0x5813,0x03);
    sccb_senddata(0x5814,0x00);
    sccb_senddata(0x5815,0x01);
    sccb_senddata(0x5816,0x03);
    sccb_senddata(0x5817,0x08);
    sccb_senddata(0x5818,0x0d);
    sccb_senddata(0x5819,0x08);
    sccb_senddata(0x581a,0x05);
    sccb_senddata(0x581b,0x06);
    sccb_senddata(0x581c,0x08);
    sccb_senddata(0x581d,0x0e);
    sccb_senddata(0x581e,0x29);
    sccb_senddata(0x581f,0x17);
    sccb_senddata(0x5820,0x11);
    sccb_senddata(0x5821,0x11);
    sccb_senddata(0x5822,0x15);
    sccb_senddata(0x5823,0x28);
    sccb_senddata(0x5824,0x46);
    sccb_senddata(0x5825,0x26);
    sccb_senddata(0x5826,0x08);
    sccb_senddata(0x5827,0x26);
    sccb_senddata(0x5828,0x64);
    sccb_senddata(0x5829,0x26);
    sccb_senddata(0x582a,0x24);
    sccb_senddata(0x582b,0x22);
    sccb_senddata(0x582c,0x24);
    sccb_senddata(0x582d,0x24);
    sccb_senddata(0x582e,0x06);
    sccb_senddata(0x582f,0x22);
    sccb_senddata(0x5830,0x40);
    sccb_senddata(0x5831,0x42);
    sccb_senddata(0x5832,0x24);
    sccb_senddata(0x5833,0x26);
    sccb_senddata(0x5834,0x24);
    sccb_senddata(0x5835,0x22);
    sccb_senddata(0x5836,0x22);
    sccb_senddata(0x5837,0x26);
    sccb_senddata(0x5838,0x44);
    sccb_senddata(0x5839,0x24);
    sccb_senddata(0x583a,0x26);
    sccb_senddata(0x583b,0x28);
    sccb_senddata(0x583c,0x42);
    sccb_senddata(0x583d,0xce);// lenc BR offset // AWB
    sccb_senddata(0x5180,0xff);// AWB B block
    sccb_senddata(0x5181,0x58);// AWB control
    sccb_senddata(0x5182,0x11);// [7:4] max local counter,0x [3:0] max fast counter
    sccb_senddata(0x5183,0x90);// AWB advanced
    sccb_senddata(0x5184,0x25);
    sccb_senddata(0x5185,0x24);
    sccb_senddata(0x5186,0x09);
    sccb_senddata(0x5187,0x09);
    sccb_senddata(0x5188,0x09);
    sccb_senddata(0x5189,0x75);
    sccb_senddata(0x518a,0x54);
    sccb_senddata(0x518b,0xe0);
    sccb_senddata(0x518c,0xb2);
    sccb_senddata(0x518d,0x42);
    sccb_senddata(0x518e,0x3d);
    sccb_senddata(0x518f,0x56);
    sccb_senddata(0x5190,0x46);
    sccb_senddata(0x5191,0xff);// AWB top limit
    sccb_senddata(0x5192,0x00);// AWB bottom limit
    sccb_senddata(0x5193,0xf0);// red limit
    sccb_senddata(0x5194,0xf0);// green limit
    sccb_senddata(0x5195,0xf0);// blue limit
    sccb_senddata(0x5196,0x03);// AWB control
    sccb_senddata(0x5197,0x02);// local limit
    sccb_senddata(0x5198,0x04);
    sccb_senddata(0x5199,0x12);
    sccb_senddata(0x519a,0x04);
    sccb_senddata(0x519b,0x00);
    sccb_senddata(0x519c,0x06);
    sccb_senddata(0x519d,0x82);
    sccb_senddata(0x519e,0x00);// AWB control // Gamma
    sccb_senddata(0x5480,0x01);// Gamma bias plus on,0x bit[0]
    sccb_senddata(0x5481,0x08);
    sccb_senddata(0x5482,0x14);
    sccb_senddata(0x5483,0x28);
    sccb_senddata(0x5484,0x51);
    sccb_senddata(0x5485,0x65);
    sccb_senddata(0x5486,0x71);
    sccb_senddata(0x5487,0x7d);
    sccb_senddata(0x5488,0x87);
    sccb_senddata(0x5489,0x91);
    sccb_senddata(0x548a,0x9a);
    sccb_senddata(0x548b,0xaa);
    sccb_senddata(0x548c,0xb8);
    sccb_senddata(0x548d,0xcd);
    sccb_senddata(0x548e,0xdd);
    sccb_senddata(0x548f,0xea);
    sccb_senddata(0x5490,0x1d);// color matrix
    sccb_senddata(0x5381,0x1e);// CMX1 for Y
    sccb_senddata(0x5382,0x5b);// CMX2 for Y
    sccb_senddata(0x5383,0x08);// CMX3 for Y
    sccb_senddata(0x5384,0x0a);// CMX4 for U
    sccb_senddata(0x5385,0x7e);// CMX5 for U
    sccb_senddata(0x5386,0x88);//
    sccb_senddata(0x5387,0x7c);// CMX7 for V
    sccb_senddata(0x5388,0x6c);// CMX8 for V
    sccb_senddata(0x5389,0x10);// CMX9 for V
    sccb_senddata(0x538a,0x01);// sign[9]
    sccb_senddata(0x538b,0x98); // sign[8:1] // UV adjust   UV
    sccb_senddata(0x5580,0x06);// saturation on,0x bit[1]
    sccb_senddata(0x5583,0x40);
    sccb_senddata(0x5584,0x10);
    sccb_senddata(0x5589,0x10);
    sccb_senddata(0x558a,0x00);
    sccb_senddata(0x558b,0xf8);
    sccb_senddata(0x501d,0x40);// enable manual offset of contrast// CIP
    sccb_senddata(0x5300,0x08);// CIP sharpen MT threshold 1
    sccb_senddata(0x5301,0x30);// CIP sharpen MT threshold 2
    sccb_senddata(0x5302,0x10);// CIP sharpen MT offset 1
    sccb_senddata(0x5303,0x00);// CIP sharpen MT offset 2
    sccb_senddata(0x5304,0x08);// CIP DNS threshold 1
    sccb_senddata(0x5305,0x30);// CIP DNS threshold 2
    sccb_senddata(0x5306,0x08);// CIP DNS offset 1
    sccb_senddata(0x5307,0x16);// CIP DNS offset 2
    sccb_senddata(0x5309,0x08);// CIP sharpen TH threshold 1
    sccb_senddata(0x530a,0x30);// CIP sharpen TH threshold 2
    sccb_senddata(0x530b,0x04);// CIP sharpen TH offset 1
    sccb_senddata(0x530c,0x06);// CIP sharpen TH offset 2
    sccb_senddata(0x5025,0x00);
    sccb_senddata(0x3008,0x02); // wake up from standby,0x bit[6]
    sccb_senddata(0x3035, 0x11); // PLL
    sccb_senddata(0x3036, 0x46); // PLL
    sccb_senddata(0x3c07, 0x08); // light meter 1 threshold [7:0]
    sccb_senddata(0x3820, 0x41); // Sensor flip off, ISP flip on
    sccb_senddata(0x3821, 0x07); // Sensor mirror on, ISP mirror on, H binning on
    sccb_senddata(0x3814, 0x31); // X INC
    sccb_senddata(0x3815, 0x31); // Y INC
    sccb_senddata(0x3800, 0x00); // HS
    sccb_senddata(0x3801, 0x00); // HS
    sccb_senddata(0x3802, 0x00); // VS
    sccb_senddata(0x3803, 0x04); // VS
    sccb_senddata(0x3804, 0x0a); // HW (HE)
    sccb_senddata(0x3805, 0x3f); // HW (HE)
    sccb_senddata(0x3806, 0x07); // VH (VE)
    sccb_senddata(0x3807, 0x9b); // VH (VE)
    sccb_senddata(0x3808, 0x02); // DVPHO  640
    sccb_senddata(0x3809, 0x80); // DVPHO
    sccb_senddata(0x380a, 0x01); // DVPVO  480
    sccb_senddata(0x380b, 0xe0); // DVPVO
    sccb_senddata(0x380c, 0x07); // HTS
    sccb_senddata(0x380d, 0x68); // HTS
    sccb_senddata(0x380e, 0x03); // VTS
    sccb_senddata(0x380f, 0xd8); // VTS
    sccb_senddata(0x3813, 0x06); // Timing Voffset
    sccb_senddata(0x3618, 0x00);
    sccb_senddata(0x3612, 0x29);
    sccb_senddata(0x3709, 0x52);
    sccb_senddata(0x370c, 0x03);
    sccb_senddata(0x3a02, 0x17); // 60Hz max exposure, night mode 5fps
    sccb_senddata(0x3a03, 0x10); // 60Hz max exposure
    sccb_senddata(0x3a14, 0x17); // 50Hz max exposure, night mode 5fps
    sccb_senddata(0x3a15, 0x10); // 50Hz max exposure
    sccb_senddata(0x4004, 0x02); // BLC 2 lines
    sccb_senddata(0x3002, 0x1c); // reset JFIFO, SFIFO, JPEG
    sccb_senddata(0x3006, 0xc3); // disable clock of JPEG2x, JPEG
    sccb_senddata(0x4713, 0x03); // JPEG mode 3
    sccb_senddata(0x4407, 0x04); // Quantization scale
    sccb_senddata(0x460b, 0x35);
    sccb_senddata(0x460c, 0x22);
    sccb_senddata(0x4837, 0x22); // DVP CLK divider
    sccb_senddata(0x3824, 0x02); // DVP CLK divider
    sccb_senddata(0x5001, 0xa3); // SDE on, scale on, UV average off, color matrix on, AWB on
    sccb_senddata(0x3503, 0x00); // AEC/AGC on

}
void MainWindow::ov5640_init_mode_2(void)
{
    sccb_senddata1(0x3103,0x11);// system clock from pad,0x bit[1]
    sccb_senddata1(0x3008,0x82);// software reset,0x bit[7]// delay 5ms
    sccb_senddata1(0x3008,0x42);// software power down,0x bit[6]
    sccb_senddata1(0x3103,0x03);// system clock from PLL,0x bit[1]
    sccb_senddata1(0x3017,0xff);// FREX,0x Vsync,0x HREF,0x PCLK,0x D[9:6] output enable
    sccb_senddata1(0x3018,0xff);// D[5:0],0x GPIO[1:0] output enable
    sccb_senddata1(0x3034,0x1A);// MIPI 10-bit
    sccb_senddata1(0x3037,0x13);// PLL root divider,0x bit[4],0x PLL pre-divider,0x bit[3:0]
    sccb_senddata1(0x3108,0x01);// PCLK root divider,0x bit[5:4],0x SCLK2x root divider,0x bit[3:2] // SCLK root divider,0x bit[1:0]
    sccb_senddata1(0x3630,0x36);
    sccb_senddata1(0x3631,0x0e);
    sccb_senddata1(0x3632,0xe2);
    sccb_senddata1(0x3633,0x12);
    sccb_senddata1(0x3621,0xe0);
    sccb_senddata1(0x3704,0xa0);
    sccb_senddata1(0x3703,0x5a);
    sccb_senddata1(0x3715,0x78);
    sccb_senddata1(0x3717,0x01);
    sccb_senddata1(0x370b,0x60);
    sccb_senddata1(0x3705,0x1a);
    sccb_senddata1(0x3905,0x02);
    sccb_senddata1(0x3906,0x10);
    sccb_senddata1(0x3901,0x0a);
    sccb_senddata1(0x3731,0x12);
    sccb_senddata1(0x3600,0x08);// VCM control
    sccb_senddata1(0x3601,0x33);// VCM control
    sccb_senddata1(0x302d,0x60);// system control
    sccb_senddata1(0x3620,0x52);
    sccb_senddata1(0x371b,0x20);
    sccb_senddata1(0x471c,0x50);
    sccb_senddata1(0x3a13,0x43);// pre-gain = 1.047x
    sccb_senddata1(0x3a18,0x00);// gain ceiling
    sccb_senddata1(0x3a19,0xf8);// gain ceiling = 15.5x
    sccb_senddata1(0x3635,0x13);
    sccb_senddata1(0x3636,0x03);
    sccb_senddata1(0x3634,0x40);
    sccb_senddata1(0x3622,0x01); // 50/60Hz detection     50/60Hz
    sccb_senddata1(0x3c01,0x34);// Band auto,0x bit[7]
    sccb_senddata1(0x3c04,0x28);// threshold low sum
    sccb_senddata1(0x3c05,0x98);// threshold high sum
    sccb_senddata1(0x3c06,0x00);// light meter 1 threshold[15:8]
    sccb_senddata1(0x3c07,0x08);// light meter 1 threshold[7:0]
    sccb_senddata1(0x3c08,0x00);// light meter 2 threshold[15:8]
    sccb_senddata1(0x3c09,0x1c);// light meter 2 threshold[7:0]
    sccb_senddata1(0x3c0a,0x9c);// sample number[15:8]
    sccb_senddata1(0x3c0b,0x40);// sample number[7:0]
    sccb_senddata1(0x3810,0x00);// Timing Hoffset[11:8]
    sccb_senddata1(0x3811,0x10);// Timing Hoffset[7:0]
    sccb_senddata1(0x3812,0x00);// Timing Voffset[10:8]
    sccb_senddata1(0x3708,0x64);
    sccb_senddata1(0x4001,0x02);// BLC start from line 2
    sccb_senddata1(0x4005,0x1a);// BLC always update
    sccb_senddata1(0x3000,0x00);// enable blocks
    sccb_senddata1(0x3004,0xff);// enable clocks
    sccb_senddata1(0x300e,0x58);// MIPI power down,0x DVP enable
    sccb_senddata1(0x302e,0x00);
    sccb_senddata1(0x4300,0x61);// RGB565
    sccb_senddata1(0x501f,0x01);// ISP RGB
    sccb_senddata1(0x440e,0x00);
    sccb_senddata1(0x5000,0xa7); // Lenc on,0x raw gamma on,0x BPC on,0x WPC on,0x CIP on // AEC target
    sccb_senddata1(0x3a0f,0x30);// stable range in high
    sccb_senddata1(0x3a10,0x28);// stable range in low
    sccb_senddata1(0x3a1b,0x30);// stable range out high
    sccb_senddata1(0x3a1e,0x26);// stable range out low
    sccb_senddata1(0x3a11,0x60);// fast zone high
    sccb_senddata1(0x3a1f,0x14);// fast zone low// Lens correction for
    sccb_senddata1(0x5800,0x23);
    sccb_senddata1(0x5801,0x14);
    sccb_senddata1(0x5802,0x0f);
    sccb_senddata1(0x5803,0x0f);
    sccb_senddata1(0x5804,0x12);
    sccb_senddata1(0x5805,0x26);
    sccb_senddata1(0x5806,0x0c);
    sccb_senddata1(0x5807,0x08);
    sccb_senddata1(0x5808,0x05);
    sccb_senddata1(0x5809,0x05);
    sccb_senddata1(0x580a,0x08);
    sccb_senddata1(0x580b,0x0d);
    sccb_senddata1(0x580c,0x08);
    sccb_senddata1(0x580d,0x03);
    sccb_senddata1(0x580e,0x00);
    sccb_senddata1(0x580f,0x00);
    sccb_senddata1(0x5810,0x03);
    sccb_senddata1(0x5811,0x09);
    sccb_senddata1(0x5812,0x07);
    sccb_senddata1(0x5813,0x03);
    sccb_senddata1(0x5814,0x00);
    sccb_senddata1(0x5815,0x01);
    sccb_senddata1(0x5816,0x03);
    sccb_senddata1(0x5817,0x08);
    sccb_senddata1(0x5818,0x0d);
    sccb_senddata1(0x5819,0x08);
    sccb_senddata1(0x581a,0x05);
    sccb_senddata1(0x581b,0x06);
    sccb_senddata1(0x581c,0x08);
    sccb_senddata1(0x581d,0x0e);
    sccb_senddata1(0x581e,0x29);
    sccb_senddata1(0x581f,0x17);
    sccb_senddata1(0x5820,0x11);
    sccb_senddata1(0x5821,0x11);
    sccb_senddata1(0x5822,0x15);
    sccb_senddata1(0x5823,0x28);
    sccb_senddata1(0x5824,0x46);
    sccb_senddata1(0x5825,0x26);
    sccb_senddata1(0x5826,0x08);
    sccb_senddata1(0x5827,0x26);
    sccb_senddata1(0x5828,0x64);
    sccb_senddata1(0x5829,0x26);
    sccb_senddata1(0x582a,0x24);
    sccb_senddata1(0x582b,0x22);
    sccb_senddata1(0x582c,0x24);
    sccb_senddata1(0x582d,0x24);
    sccb_senddata1(0x582e,0x06);
    sccb_senddata1(0x582f,0x22);
    sccb_senddata1(0x5830,0x40);
    sccb_senddata1(0x5831,0x42);
    sccb_senddata1(0x5832,0x24);
    sccb_senddata1(0x5833,0x26);
    sccb_senddata1(0x5834,0x24);
    sccb_senddata1(0x5835,0x22);
    sccb_senddata1(0x5836,0x22);
    sccb_senddata1(0x5837,0x26);
    sccb_senddata1(0x5838,0x44);
    sccb_senddata1(0x5839,0x24);
    sccb_senddata1(0x583a,0x26);
    sccb_senddata1(0x583b,0x28);
    sccb_senddata1(0x583c,0x42);
    sccb_senddata1(0x583d,0xce);// lenc BR offset // AWB
    sccb_senddata1(0x5180,0xff);// AWB B block
    sccb_senddata1(0x5181,0x58);// AWB control
    sccb_senddata1(0x5182,0x11);// [7:4] max local counter,0x [3:0] max fast counter
    sccb_senddata1(0x5183,0x90);// AWB advanced
    sccb_senddata1(0x5184,0x25);
    sccb_senddata1(0x5185,0x24);
    sccb_senddata1(0x5186,0x09);
    sccb_senddata1(0x5187,0x09);
    sccb_senddata1(0x5188,0x09);
    sccb_senddata1(0x5189,0x75);
    sccb_senddata1(0x518a,0x54);
    sccb_senddata1(0x518b,0xe0);
    sccb_senddata1(0x518c,0xb2);
    sccb_senddata1(0x518d,0x42);
    sccb_senddata1(0x518e,0x3d);
    sccb_senddata1(0x518f,0x56);
    sccb_senddata1(0x5190,0x46);
    sccb_senddata1(0x5191,0xff);// AWB top limit
    sccb_senddata1(0x5192,0x00);// AWB bottom limit
    sccb_senddata1(0x5193,0xf0);// red limit
    sccb_senddata1(0x5194,0xf0);// green limit
    sccb_senddata1(0x5195,0xf0);// blue limit
    sccb_senddata1(0x5196,0x03);// AWB control
    sccb_senddata1(0x5197,0x02);// local limit
    sccb_senddata1(0x5198,0x04);
    sccb_senddata1(0x5199,0x12);
    sccb_senddata1(0x519a,0x04);
    sccb_senddata1(0x519b,0x00);
    sccb_senddata1(0x519c,0x06);
    sccb_senddata1(0x519d,0x82);
    sccb_senddata1(0x519e,0x00);// AWB control // Gamma
    sccb_senddata1(0x5480,0x01);// Gamma bias plus on,0x bit[0]
    sccb_senddata1(0x5481,0x08);
    sccb_senddata1(0x5482,0x14);
    sccb_senddata1(0x5483,0x28);
    sccb_senddata1(0x5484,0x51);
    sccb_senddata1(0x5485,0x65);
    sccb_senddata1(0x5486,0x71);
    sccb_senddata1(0x5487,0x7d);
    sccb_senddata1(0x5488,0x87);
    sccb_senddata1(0x5489,0x91);
    sccb_senddata1(0x548a,0x9a);
    sccb_senddata1(0x548b,0xaa);
    sccb_senddata1(0x548c,0xb8);
    sccb_senddata1(0x548d,0xcd);
    sccb_senddata1(0x548e,0xdd);
    sccb_senddata1(0x548f,0xea);
    sccb_senddata1(0x5490,0x1d);// color matrix
    sccb_senddata1(0x5381,0x1e);// CMX1 for Y
    sccb_senddata1(0x5382,0x5b);// CMX2 for Y
    sccb_senddata1(0x5383,0x08);// CMX3 for Y
    sccb_senddata1(0x5384,0x0a);// CMX4 for U
    sccb_senddata1(0x5385,0x7e);// CMX5 for U
    sccb_senddata1(0x5386,0x88);//
    sccb_senddata1(0x5387,0x7c);// CMX7 for V
    sccb_senddata1(0x5388,0x6c);// CMX8 for V
    sccb_senddata1(0x5389,0x10);// CMX9 for V
    sccb_senddata1(0x538a,0x01);// sign[9]
    sccb_senddata1(0x538b,0x98); // sign[8:1] // UV adjust   UV
    sccb_senddata1(0x5580,0x06);// saturation on,0x bit[1]
    sccb_senddata1(0x5583,0x40);
    sccb_senddata1(0x5584,0x10);
    sccb_senddata1(0x5589,0x10);
    sccb_senddata1(0x558a,0x00);
    sccb_senddata1(0x558b,0xf8);
    sccb_senddata1(0x501d,0x40);// enable manual offset of contrast// CIP
    sccb_senddata1(0x5300,0x08);// CIP sharpen MT threshold 1
    sccb_senddata1(0x5301,0x30);// CIP sharpen MT threshold 2
    sccb_senddata1(0x5302,0x10);// CIP sharpen MT offset 1
    sccb_senddata1(0x5303,0x00);// CIP sharpen MT offset 2
    sccb_senddata1(0x5304,0x08);// CIP DNS threshold 1
    sccb_senddata1(0x5305,0x30);// CIP DNS threshold 2
    sccb_senddata1(0x5306,0x08);// CIP DNS offset 1
    sccb_senddata1(0x5307,0x16);// CIP DNS offset 2
    sccb_senddata1(0x5309,0x08);// CIP sharpen TH threshold 1
    sccb_senddata1(0x530a,0x30);// CIP sharpen TH threshold 2
    sccb_senddata1(0x530b,0x04);// CIP sharpen TH offset 1
    sccb_senddata1(0x530c,0x06);// CIP sharpen TH offset 2
    sccb_senddata1(0x5025,0x00);
    sccb_senddata1(0x3008,0x02); // wake up from standby,0x bit[6]
    sccb_senddata1(0x3035, 0x11); // PLL
    sccb_senddata1(0x3036, 0x46); // PLL
    sccb_senddata1(0x3c07, 0x08); // light meter 1 threshold [7:0]
    sccb_senddata1(0x3820, 0x41); // Sensor flip off, ISP flip on
    sccb_senddata1(0x3821, 0x07); // Sensor mirror on, ISP mirror on, H binning on
    sccb_senddata1(0x3814, 0x31); // X INC
    sccb_senddata1(0x3815, 0x31); // Y INC
    sccb_senddata1(0x3800, 0x00); // HS
    sccb_senddata1(0x3801, 0x00); // HS
    sccb_senddata1(0x3802, 0x00); // VS
    sccb_senddata1(0x3803, 0x04); // VS
    sccb_senddata1(0x3804, 0x0a); // HW (HE)
    sccb_senddata1(0x3805, 0x3f); // HW (HE)
    sccb_senddata1(0x3806, 0x07); // VH (VE)
    sccb_senddata1(0x3807, 0x9b); // VH (VE)
    sccb_senddata1(0x3808, 0x02); // DVPHO  640
    sccb_senddata1(0x3809, 0x80); // DVPHO
    sccb_senddata1(0x380a, 0x01); // DVPVO  480
    sccb_senddata1(0x380b, 0xe0); // DVPVO
    sccb_senddata1(0x380c, 0x07); // HTS
    sccb_senddata1(0x380d, 0x68); // HTS
    sccb_senddata1(0x380e, 0x03); // VTS
    sccb_senddata1(0x380f, 0xd8); // VTS
    sccb_senddata1(0x3813, 0x06); // Timing Voffset
    sccb_senddata1(0x3618, 0x00);
    sccb_senddata1(0x3612, 0x29);
    sccb_senddata1(0x3709, 0x52);
    sccb_senddata1(0x370c, 0x03);
    sccb_senddata1(0x3a02, 0x17); // 60Hz max exposure, night mode 5fps
    sccb_senddata1(0x3a03, 0x10); // 60Hz max exposure
    sccb_senddata1(0x3a14, 0x17); // 50Hz max exposure, night mode 5fps
    sccb_senddata1(0x3a15, 0x10); // 50Hz max exposure
    sccb_senddata1(0x4004, 0x02); // BLC 2 lines
    sccb_senddata1(0x3002, 0x1c); // reset JFIFO, SFIFO, JPEG
    sccb_senddata1(0x3006, 0xc3); // disable clock of JPEG2x, JPEG
    sccb_senddata1(0x4713, 0x03); // JPEG mode 3
    sccb_senddata1(0x4407, 0x04); // Quantization scale
    sccb_senddata1(0x460b, 0x35);
    sccb_senddata1(0x460c, 0x22);
    sccb_senddata1(0x4837, 0x22); // DVP CLK divider
    sccb_senddata1(0x3824, 0x02); // DVP CLK divider
    sccb_senddata1(0x5001, 0xa3); // SDE on, scale on, UV average off, color matrix on, AWB on
    sccb_senddata1(0x3503, 0x00); // AEC/AGC on

}

void MainWindow::cam_vdma0_init()
{
    int status;
    int hsize,stride;
    VDMA0_fd = ::open(CAMERA_VDMA_0_DEVICE, O_RDWR, 0);
    if (VDMA0_fd < 0) {
            printf("Open %s failed\n", CAMERA_VDMA_0_DEVICE);
           // return -1;
    }
    status = 1;//0->cirbuffer; 1->park;
    ::ioctl(VDMA0_fd, WR_S2MM_WROKMODE, &status);
    CameraBuf = (unsigned char*)::mmap(NULL, VDMA_BUF_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, VDMA0_fd, 0);
    ::ioctl(VDMA0_fd, START_S2MM_RX, &status);
    hsize=640*3;
    stride=640*4;
    ::ioctl(VDMA0_fd, RD_S2MM_HSIZE, &hsize);
    ::ioctl(VDMA0_fd, RD_S2MM_FRAME_STRIDE, &stride);
     printf("Open cam1 success!!!\n");
}

void MainWindow::cam_vdma0_stop()
{
   int status;
   ::ioctl(VDMA0_fd, STOP_AXIVDMA_RX, &status);
   status=0;
   while(status!=1){
       ::ioctl(VDMA0_fd, AXIVDMA_RX_STATUS, &status);
       usleep(5000);//sleep 50 ms
   }
   ::munmap(CameraBuf, VDMA_BUF_SIZE);
   ::close(VDMA0_fd);
}
void MainWindow::cam_vdma1_init()
{
    int status;
    int hsize,stride;
    VDMA1_fd = ::open(CAMERA_VDMA_1_DEVICE, O_RDWR, 0);
    if (VDMA1_fd < 0) {
            printf("Open %s failed\n", CAMERA_VDMA_1_DEVICE);
    }
    status = 1;//0->cirbuffer; 1->park;
    ::ioctl(VDMA1_fd, WR_S2MM_WROKMODE, &status);
    CameraBuf1 = (unsigned char*)::mmap(NULL, VDMA_BUF_SIZE, PROT_READ|PROT_WRITE, MAP_SHARED, VDMA1_fd, 0);
    ::ioctl(VDMA1_fd, START_S2MM_RX, &status);
    hsize=640*4;
    stride=640*4;
    ::ioctl(VDMA1_fd, RD_S2MM_HSIZE, &hsize);
    ::ioctl(VDMA1_fd, RD_S2MM_FRAME_STRIDE, &stride);

    printf("Open cam2 success!!!\n");
}
void MainWindow::cam_vdma1_stop()
{
   int status;
   ::ioctl(VDMA1_fd, STOP_AXIVDMA_RX, &status);
   status=0;
   while(status!=1){
       ::ioctl(VDMA1_fd, AXIVDMA_RX_STATUS, &status);
       usleep(5000);//sleep 50 ms
   }
   ::munmap(CameraBuf1, VDMA_BUF_SIZE);
   ::close(VDMA1_fd);
}
void MainWindow::DDR3toRGB888_0( void *src)
{
    unsigned char * pointer;
    unsigned char *imagebits24;
    pointer = (unsigned char *)src;
    imagebits24 = imageRGB888_0.scanLine(0);
    memcpy(imagebits24,pointer,640*480*3);
    memcpy(imagebits24,pointer,640*480*3);
    memcpy(imagebits24,pointer,640*480*3);
}
void MainWindow::DDR3toRGB888_1( void *src)
{
    unsigned char * pointer;
    unsigned char *imagebits24;
    pointer = (unsigned char *)src;
    imagebits24 = imageRGB888_1.scanLine(0);
    memcpy(imagebits24,pointer,640*480*3);
    memcpy(imagebits24,pointer,640*480*3);
    memcpy(imagebits24,pointer,640*480*3);
}

void MainWindow::Init_Qimage_parameter0(int height,int width)
{
    //for grey iamge
    //int k;
    QVector<QRgb> colorTable;

    imageGRY8=QImage(width,height,QImage::Format_Indexed8);
    for(int k=0;k<256;++k)
    {
        colorTable.push_back( qRgb(k,k,k) );
    }
    imageGRY8.setColorTable(colorTable);
    //RGB888
    imageRGB888_0=QImage(width,height,QImage::Format_RGB888);
}
void MainWindow::Init_Qimage_parameter1(int height,int width)
{
    //for grey iamge
    //int k;
    QVector<QRgb> colorTable;

    imageGRY7=QImage(width,height,QImage::Format_Indexed8);
    for(int k=0;k<256;++k)
    {
        colorTable.push_back( qRgb(k,k,k) );
    }
    imageGRY7.setColorTable(colorTable);
    //RGB888
    imageRGB888_1=QImage(width,height,QImage::Format_RGB888);
}

void MainWindow::slot_timer_0()
{
    int offset;
    //capture image
    ::ioctl(VDMA0_fd, READ_BUFFER, &offset);
    DDR3toRGB888_0(CameraBuf+offset);
    //process image
    ui->cam_label_1->resize(640,480);
    ui->cam_label_1->setPixmap(QPixmap::fromImage(imageRGB888_0).scaled(ui->cam_label_1->size()));

}
void MainWindow::slot_timer_1()
{
    int offset;
    //capture image
    ::ioctl(VDMA1_fd, READ_BUFFER, &offset);
    DDR3toRGB888_1(CameraBuf1+offset);
    //process image
    ui->cam_label_2->resize(640,480);
    ui->cam_label_2->setPixmap(QPixmap::fromImage(imageRGB888_1).scaled(ui->cam_label_2->size()));

}

void MainWindow::on_open_video_pushButton_clicked()
{
    Init_Qimage_parameter0(480,640);
    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(slot_timer_0()));
    timer->start(30);
    ov5640_init_mode_1();
    cam_vdma0_init();
    printf("Open cam1 success!!!\n");

    Init_Qimage_parameter1(480,640);
    timer1 = new QTimer(this);
    connect(timer1, SIGNAL(timeout()), this, SLOT(slot_timer_1()));
    timer1->start(30);
    ui->close_video_pushButton->setDisabled(false);
    ui->open_video_pushButton->setDisabled(true);
    ui->Snapshot->setDisabled(false);
    ui->Stitch_pushButton->setDisabled(true);
    ov5640_init_mode_2();
    cam_vdma1_init();
    printf("Open cam2 success!!!\n");
}

void MainWindow::on_close_video_pushButton_clicked()
{
    timer->stop();
    timer1->stop();
    ui->close_video_pushButton->setDisabled(true);
    ui->open_video_pushButton->setDisabled(false);
    ui->Snapshot->setDisabled(true);
    ui->orb_pushButton->setDisabled(true);
    ui->Stitch_pushButton->setDisabled(true);
    cam_vdma0_stop();
    cam_vdma1_stop();
}

void MainWindow::on_Snapshot_clicked()
{
    QString pic_video1="/home/video1.jpg";
    QString pic_video2="/home/video2.jpg";
    int offset1;
    int offset2;
    //capture image
    ::ioctl(VDMA0_fd, READ_BUFFER, &offset1);
    ::ioctl(VDMA1_fd, READ_BUFFER, &offset2);
    DDR3toRGB888_0(CameraBuf+offset1);
    DDR3toRGB888_1(CameraBuf1+offset2);
    imageRGB888_0.save(pic_video1,"jpg",100);
     printf("saved pic_video1 success!!!\n");
    imageRGB888_1.save(pic_video2,"jpg",100);
     printf("saved pic_video2 success!!!\n");
     ui->close_video_pushButton->setDisabled(false);
     ui->open_video_pushButton->setDisabled(true);
     ui->Snapshot->setDisabled(true);
     ui->orb_pushButton->setDisabled(false);
     ui->Stitch_pushButton->setDisabled(false);
}

void MainWindow::on_Stitch_pushButton_clicked()
{

    QTime time0;
    int time_cost0;
    Mat image01 = imread("/home/video1.jpg");
    Mat image02 = imread("/home/video2.jpg");
    Mat image3;

     vector<Mat> imgs;
     imgs.push_back(image01);
     imgs.push_back(image02);

     time0.start();
     stitch_orb(imgs,image3);
     time_cost0 = time0.elapsed();

     imwrite("/home/outimageorb.jpg",image3);
     printf("saved outimageorb success!!!\n");
     image_show(ui->label, image3);

    //    time0.start();
//    image1 = stitch_sift(image01,image02);
    //    time_cost0 = time0.elapsed();

//     imwrite("/home/outimagesift.jpg",image1);
//     printf("saved pic1 success!!!\n");
    //    time1.start();
//    image2 = stitch_surf(image01,image02);
    //    time_cost1 = time1.elapsed();

//     imwrite("/home/outimagesurf.jpg",image2);
//     printf("saved pic2 success!!!\n");

    QString fmt("orb 1 :%2ms");
    QString str = fmt.arg(time_cost0);
    ui->label_5->setText(str);

    ui->close_video_pushButton->setDisabled(false);
    ui->open_video_pushButton->setDisabled(true);
    ui->Snapshot->setDisabled(false);
    ui->orb_pushButton->setDisabled(false);
    ui->Stitch_pushButton->setDisabled(true);
}

void MainWindow::on_orb_pushButton_clicked()
{
        QTime time0;
        int time_cost0;
        string pic1="/home/video1.jpg";
        string pic2="/home/video2.jpg";
        Mat image;
        time0.start();
        image = stitch_orb_surf(pic1,pic2);
        time_cost0 = time0.elapsed();
        imwrite("/home/outimage_orb_surf.jpg",image);
        printf("saved outimage_orb_surf success!!!\n");
        image_show(ui->label, image);

        QString fmt("orb cost:%2ms");
        QString str = fmt.arg(time_cost0);
        ui->label_6->setText(str);

        ui->close_video_pushButton->setDisabled(false);
        ui->open_video_pushButton->setDisabled(true);
        ui->Snapshot->setDisabled(false);
        ui->orb_pushButton->setDisabled(true);
        ui->Stitch_pushButton->setDisabled(false);
}
