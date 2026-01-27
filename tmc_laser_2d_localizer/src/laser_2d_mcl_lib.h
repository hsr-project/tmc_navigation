#ifndef LASER_2D_MCL_LIB_H_
#define LASER_2D_MCL_LIB_H_

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <math.h>

/* True or False */
#ifndef TRUE
#define TRUE    1
#endif
#ifndef FALSE
#define FALSE   0
#endif
#ifndef NULL
#define	NULL	(void*)0
#endif

/* Return value */
#define ICSLAM_D_RET_SUCCESS            1               /* 成功 (Success) */
#define ICSLAM_D_RET_NONE               0               /* 続く (Continue) */
#define ICSLAM_D_RET_ERROR              -1              /* その他エラー (ERROR) */
#define ICSLAM_D_RET_PARA               -2              /* パラメータエラー (PARAMETER ERROR) */
#define ICSLAM_D_RET_SYS                -3              /* システムエラー (SYSTEM ERROR) [ex. kernel call] */
#define ICSLAM_D_RET_WAIT               -4              /* ちょっと待ってね (Wait just for a moment) */
#define ICSLAM_D_RET_MAXLOCK            -5              /* 最大ロック要求数越え ( The number of the maximum lock demands was encoded) */
#define ICSLAM_D_RET_UNMATCK_PID        -6              /* セマフォをロックしていないプロセスが開放要求した。 */
#define ICSLAM_D_RET_ALREADY_OPEN  ICSLAM_D_RET_SUCCESS /* 既にオープン済み(Already open) */

/* Map */
#define ICSLAM_D_MAX_LENGTH             5000            /* [pixel] ( 100 ) */
#define ICSLAM_D_MAX_WIDTH              5000            /* [pixel] ( 100 ) */
#define ICSLAM_D_UNEXPLORED             0               /* 距離マップの未踏の領域の値 */
#define ICSLAM_D_PATH_LENGTH            1024            /* パス点数(Points of path) */
#define ICSLAM_D_PATH_ORDER             2               /* 経路の次元(Order of path) */

/* localization */
#define ICSLAM_D_MAX_NUM_PARTICLES      2000
#define ICSLAM_D_NUM_GLOBAL_PARTICLES   2000
#define ICSLAM_D_NUM_TRACKING_PARTICLES 1000

/* laser */
//レーザセンサのパラメータはクライアントが動的に与えるようにしたい。要修正。
#define ICSLAM_D_NUM_LASER              307200              //TOP-URG水平スキャン時は721、kinectの場合はVGAサイズ307200(640*480)とする
#define ICSLAM_F_LASER_RESOLUTION       (0.25*M_PI/180.0)
#define ICSLAM_F_LASER_OFFSET_X         250.0               /* for H1S */
#define ICSLAM_F_LASER_OFFSET_Y         0.0                 /* [mm] this should be changed */
#define ICSLAM_F_LASER_START_ANGLE      (-90.0*M_PI/180.0)  /* [rad] */
#define ICSLAM_F_MAX_LASER_USE_RANGE    (62000.0)           /*[mm]*/

/* robot parameters */
#define ICSLAM_F_WHEEL_TREAD            (500.0)             /* mm for H1S */
#define ICSLAM_F_WHEEL_RADIUS           (100.0)             /* mm for H1S */


#define ICSLAM_D_RET_GLOBAL             10
#define ICSLAM_D_RET_USE_ODOM           11
#define ICSLAM_D_MAX_NUM_GRID           ICSLAM_D_MAX_LENGTH*ICSLAM_D_MAX_WIDTH
#define ICSLAM_D_MAX_NUM_LASER          ICSLAM_D_NUM_LASER

/*---------------type definition----------------*/

/**
 * @brief
 */
typedef struct {
  double f_x;     /* [m] */
  double f_y;     /* [m] */
  double f_theta; /* [rad] */
} icSlam_tagOrientedPoint;
  
/**
 * @brief
 */
typedef struct {
  double f_x; /* [m] */
  double f_y; /* [m] */
} icSlam_tagPoint;
  
/**
 * @brief
 */
typedef struct {
  int32_t              d_x;
  int32_t              d_y;
} icSlam_tagIntPoint;
  
/**
 * @brief  レーザデータ(laser data)
 */
typedef struct {
  double f_data[ICSLAM_D_NUM_LASER];
} icSlam_tagLaserData;
  
/**
 *  @brief  距離マップデータ(distance map data)
 */
typedef struct {
  int32_t d_distance[ICSLAM_D_MAX_WIDTH][ICSLAM_D_MAX_LENGTH];
} icSlam_tagDistmapData;
  
/**
 * @brief  マップサイズデータ(map size data)
 */
typedef struct {
  int32_t    d_width; /* グリッドサイズ */
  int32_t    d_height; /* グリッドサイズ */
  double f_resolution; /* [m] */
  double f_threshold; /* [m] */
} icSlam_tagMapsizeData;

/**
 * @brief  パーティクル構造体
 */
typedef struct {
  icSlam_tagOrientedPoint  t_pose;
  double           f_weight;
  double           f_logWeight;
} icSlam_tagParticle;

/**
 * @brief  パーティクルセット構造体
 */
typedef struct {
  icSlam_tagParticle  t_particle[ICSLAM_D_NUM_TRACKING_PARTICLES];
  int32_t              d_numPart;
}icSlam_tagParticleSet;

/**
 * @brief  レーザの端点のxy座標を表す．グローバル座標系でなくロボット座標系．
 */
typedef struct {
  icSlam_tagPoint     t_laser[ICSLAM_D_MAX_NUM_LASER];
  int32_t              d_numLaser;
} icSlam_tagRangeXY;

icSlam_tagOrientedPoint icSlam_Ft_absoluteDiff (icSlam_tagOrientedPoint t_oldPose,
                                                   icSlam_tagOrientedPoint t_newPose);
icSlam_tagOrientedPoint icSlam_Ft_absoluteSum (icSlam_tagOrientedPoint t_pose,
                                                  icSlam_tagOrientedPoint t_dpose);
int32_t             icSlam_Fd_predict (icSlam_tagOrientedPoint t_dPose);
int32_t             icSlam_Fd_killRingOut (uint8_t* pd_distmap,
                                             const icSlam_tagOrientedPoint* pt_estpose,
                                             double f_gx,
                                             double f_gy);
int32_t             icSlam_Fd_prepareLikelihood (const int32_t* pd_useLaser,
                                                icSlam_tagLaserData* pt_laser);
double          icSlam_Ff_likelihood (uint8_t* pd_distmap,
                                            double f_gx,
					 double f_gy);
int32_t             icSlam_Fd_normalizeWeights (void);
int32_t             icSlam_Fd_checkNeff (double f_neff_threshold);
int32_t             icSlam_Fd_resample (void);
int32_t             icSlam_Fd_getExpectedPose (icSlam_tagOrientedPoint* pt_pose);
int32_t             icSlam_Fd_setMapSize (int32_t d_width,
                                            int32_t d_height,
                                            double f_resolution,
					 double f_threshold,
					 double f_filtering);
int32_t             icSlam_Fd_setInitParticles (icSlam_tagOrientedPoint t_initPose,
                                               int32_t d_numParticles,
                                               double f_standard_deviation_xy,
                                               double f_standard_deviation_theta);
int32_t             icSlam_Fd_checkLost (void);
int32_t             icSlam_Fd_setMotionNoise (double f_noiseSxy,
                                                double f_noiseStr,
                                                double f_noiseSrr,
                                                double f_noiseStt,
                                                double f_noiseSrt);
double          icSlam_Ff_getNeff (void);
void               icSlam_F_fileClose(void);
void               icSlam_F_getParticle( icSlam_tagParticleSet*  pt_ptgPartSet);
int32_t           icSlam_Fd_semcreate(void);
void               icSlam_F_semdelete( void );
void               icSlam_F_setJrtDebug( void );
int32_t             icSlam_Fd_setRangePoints(const icSlam_tagRangeXY* pt_points);

int32_t              icSlam_Fd_GetParticlePositions(icSlam_tagParticleSet* pt_particle);

#endif
