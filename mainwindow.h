#ifndef MAINWINDOW_H
#define MAINWINDOW_H
//qt header file
#include <QMainWindow>
#include<QFileDialog>
#include<QPixmap>
#include<QImage>
#include<QLabel>
#include <QTimer>
#include <QThread>
//linux header file
#include <string.h>
#include <assert.h>
#include <getopt.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include "stitch.h"
#include "stitch_orb_surf.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



#define CAMERA_VDMA_0_DEVICE "/dev/axi-vdma-cam0"
#define CAMERA_VDMA_1_DEVICE "/dev/axi-vdma-cam1"

#define CAMERA_ov5640_0_DEVICE "/dev/sccb.cam0ov5640"
#define CAMERA_ov5640_1_DEVICE "/dev/sccb.cam1ov5640"


#define VDMA_BUF_SIZE  32*1024*1024

#define RD_S2MM_VSIZE _IOR('h',1,int)//vsize   480
#define RD_S2MM_HSIZE _IOR('h',2,int)//hsize   640*3 => 24bit  640*4 => 32bit
#define RD_S2MM_FRAME_STRIDE _IOR('h',3,int)//frame stride  640*3 => 24bit 640*4 => 32bit
#define WR_S2MM_VSIZE _IOW('h',4,int)
#define WR_S2MM_HSIZE _IOW('h',5,int)
#define WR_S2MM_FRAME_STRIDE _IOR('h',6,int)
#define START_MM2S_TX _IOR('h',7,int)
#define START_S2MM_RX _IOR('h',8,int)
#define READ_BUFFER  _IOR('h',9,int)
#define WRITE_BUFFER  _IOW('h',10,int)
#define AXIVDMA_TX_STATUS _IOR('h',11,int)
#define AXIVDMA_RX_STATUS _IOR('h',12,int)
#define STOP_AXIVDMA_TX _IOW('h',13,int)
#define STOP_AXIVDMA_RX _IOW('h',14,int)
#define RPARK_PTR_RD _IOR('h',15,int)
#define RPARK_PTR_WR _IOR('h',16,int)
#define RD_MM2S_VSIZE _IOR('h',17,int)
#define RD_MM2S_HSIZE _IOR('h',18,int)
#define RD_MM2S_FRAME_STRIDE _IOR('h',19,int)
#define WR_MM2S_VSIZE _IOW('h',20,int)
#define WR_MM2S_HSIZE _IOW('h',21,int)
#define WR_MM2S_FRAME_STRIDE _IOR('h',22,int)
#define WR_MM2S_WROKMODE _IOW('h',23,int)
#define WR_S2MM_WROKMODE _IOW('h',24,int)
#define RD_MM2S_WROKMODE _IOR('h',25,int)
#define RD_S2MM_WROKMODE _IOR('h',26,int)

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
   int  VDMA0_fd;
   int  VDMA1_fd;
   int  cam0_fd;
   int  cam1_fd;

   unsigned char *CameraBuf;
   unsigned char *CameraBuf1;

   unsigned char *CameraBuf_pic1;
   unsigned char *CameraBuf_pic2;

   void image_show(QLabel *label, Mat image);

   void sccb_senddata(unsigned short addr,unsigned char value);
   void sccb_senddata1(unsigned short addr,unsigned char value);
   void ov5640_init_mode_1(void);
   void ov5640_init_mode_2(void);

   void DDR3toRGB888_0( void *src);
   void DDR3toRGB888_1( void *src);
   void cam_vdma0_init();
   void cam_vdma0_stop();
   void cam_vdma1_init();
   void cam_vdma1_stop();

   void Init_Qimage_parameter0(int height,int width);
   void Init_Qimage_parameter1(int height,int width);
private slots:
    void slot_timer_0();
    void slot_timer_1();

    void on_open_video_pushButton_clicked();
    void on_close_video_pushButton_clicked();

    void on_Snapshot_clicked();

    void on_Stitch_pushButton_clicked();

    void on_orb_pushButton_clicked();

private:
    Ui::MainWindow *ui;
    QTimer *timer;
    QTimer *timer1;
    QImage imageGRY8;
    QImage imageGRY7;
    QImage imageRGB888_0;
    QImage imageRGB888_1;

};

#endif // MAINWINDOW_H
