/**
 * @file laser_2d_mcl_lib.c
 * @brief MCL関数ライブラリ
 * @par
 * FSMのときに使っていたlocalizer.cと同じです
 * @attention ROSやFSMのようなプラットフォームに依存するコードを書かないこと
 */

/*----------------include-----------------------*/
#include <float.h>
#include <assert.h>
#include <stdint.h>
#include "laser_2d_mcl_lib.h"

/*---------------definition---------------------*/
#define ICSLAM_F_SXY_DF                     (0.05)
#define ICSLAM_F_STR_DF                     (100.0)
#define ICSLAM_F_SRR_DF                     (0.10)
#define ICSLAM_F_STT_DF                     (0.10)
#define ICSLAM_F_SRT_DF                     (0.00010)

/*---------------global variables---------------*/
static icSlam_tagParticleSet    icSlam_st_PartSet = {{{{(double)0.0,(double)0.0,(double)0.0}
                                                     ,(double)0.0,(double)0.0}}
                                                     ,(int32_t)0};
static icSlam_tagMapsizeData    icSlam_st_mapSize   = { 0,0,0.0,0.0};
static icSlam_tagIntPoint       icSlam_st_intMapOffset = { 0,0};      /* origin of map */
static icSlam_tagRangeXY        icSlam_st_rangeXY   = {{{ 0.0,0.0}},0};
static double                icSlam_sf_sxy    = ICSLAM_F_SXY_DF;
static double                icSlam_sf_str    = ICSLAM_F_STR_DF;
static double                icSlam_sf_srr    = ICSLAM_F_SRR_DF;
static double                icSlam_sf_stt    = ICSLAM_F_STT_DF;
static double                icSlam_sf_srt    = ICSLAM_F_SRT_DF;
static double                icSlam_sf_maxLaserRange = ICSLAM_F_MAX_LASER_USE_RANGE; /*20110620 asahara 今後不使用*/
static double                icSlam_sf_neff   = 0.0;
static double                icSlam_sf_filtering = 0.0; 

/*------internal function prototype-------------*/
static icSlam_tagIntPoint Ft_world2map (icSlam_tagPoint t_local_pose);
static double Ff_logLikelihood      (const uint8_t* pd_distmap,
					icSlam_tagOrientedPoint t_local_pose,
					double* f_reject_laser);
static int32_t    Fd_computeNeff        (double f_neff_threshold);
static icSlam_tagOrientedPoint Ft_drawMotion (icSlam_tagOrientedPoint p,
                                                 icSlam_tagOrientedPoint dPose);
static icSlam_tagOrientedPoint Ft_absoluteSum(icSlam_tagOrientedPoint p1,
                                                 icSlam_tagOrientedPoint p2);
static double Ff_sampleGaussian  (double f_sd);
static int32_t    Fd_normalizeWeights(double f_min_weight);
static int32_t    Fd_resample         (int32_t* const pd_indexes, int32_t d_size);
static int32_t    Fd_repeatIndexes   (icSlam_tagParticleSet* const pt_children,
                                        const icSlam_tagParticleSet* pt_parents,
                                        const int32_t* pd_indexes);
static int32_t    Fd_isInside         (icSlam_tagIntPoint t_int_point);
static double Ff_pmPI              (double theta);

/**
 * @brief グローバル座標から地図座標へ変換
 * @param t_local_pose ローカル座標系での位置
 * @return 地図上のグリッド位置
 */
static icSlam_tagIntPoint Ft_world2map(const icSlam_tagPoint t_local_pose)
{
  icSlam_tagIntPoint t_ip;

  /* 0割防止 */
  assert(icSlam_st_mapSize.f_resolution > DBL_EPSILON);

  t_ip.d_x = (int32_t)( (t_local_pose.f_x)/icSlam_st_mapSize.f_resolution);
  t_ip.d_y = (int32_t)( (t_local_pose.f_y)/icSlam_st_mapSize.f_resolution);

  return t_ip;
}

/**
 * @brief 二つのオドメトリ姿勢の差分（移動量）を求める
 * @param t_oldPose 基準オドメトリ
 * @param t_newPose 移動後のオドメトリ
 * @par
 * 移動量はt_oldPoseを原点とする．
 */
icSlam_tagOrientedPoint icSlam_Ft_absoluteDiff (const icSlam_tagOrientedPoint t_oldPose,
                                                   const icSlam_tagOrientedPoint t_newPose)
{
  icSlam_tagOrientedPoint t_p;
  icSlam_tagOrientedPoint t_dPose;
  double f_sinVal = 0.0;
  double f_cosVal = 0.0;
  double f_theta = 0.0;

  f_theta = t_newPose.f_theta - t_oldPose.f_theta;

  t_p.f_x = t_newPose.f_x - t_oldPose.f_x;
  t_p.f_y = t_newPose.f_y - t_oldPose.f_y;
  f_sinVal = sin(f_theta);
  f_cosVal = cos(f_theta);
  t_p.f_theta = atan2 (f_sinVal, f_cosVal);

  f_sinVal = sin(t_oldPose.f_theta);
  f_cosVal = cos(t_oldPose.f_theta);

  t_dPose.f_x =  f_cosVal*t_p.f_x + f_sinVal*t_p.f_y;
  t_dPose.f_y = -f_sinVal*t_p.f_x + f_cosVal*t_p.f_y;
  t_dPose.f_theta = t_p.f_theta;

  return t_dPose;
}

/**
 * @brief オドメトリ姿勢に移動量を足しこむ
 */
icSlam_tagOrientedPoint icSlam_Ft_absoluteSum (icSlam_tagOrientedPoint t_pose,
                                                  icSlam_tagOrientedPoint t_dpose)
{
  icSlam_tagOrientedPoint t_retp;
  t_retp = Ft_absoluteSum (t_pose, t_dpose);
  return t_retp;
}

/**
 * @brief パーティクルを移動させる
 * @param t_dPose オドメトリ移動量
 */
int32_t icSlam_Fd_predict (icSlam_tagOrientedPoint t_dPose)
{
  int32_t d_i;
  for (d_i=0; d_i<icSlam_st_PartSet.d_numPart; d_i++){
    icSlam_st_PartSet.t_particle[d_i].t_pose = Ft_drawMotion (icSlam_st_PartSet.t_particle[d_i].t_pose, t_dPose);
  }
  return TRUE;
}

/**
 * @brief 地図からはみ出た or 未観測領域に入った or 壁の中に入った，
 * を満たすと、ベストパーティクルの位置に再生する
 * @param pd_distmap 地図データのポインタ
 * @param pt_estpose パーティクルの再生位置
 * @param f_gx 地図のx軸オフセット(m)
 * @param f_gy 地図のy軸オフセット(m)
*/
int32_t icSlam_Fd_killRingOut (uint8_t* pd_distmap,
			      const icSlam_tagOrientedPoint* const pt_estpose,
                              const double f_gx,
			      const double f_gy)
{
  icSlam_tagPoint t_local_pose;
  int32_t d_flagKill = (int32_t)0;
  int32_t d_i;
  //double f_x,f_y;
  
  /* if a particle is in unseen area, kill it */
  for (d_i=0; d_i<icSlam_st_PartSet.d_numPart; d_i++){

    t_local_pose.f_x = icSlam_st_PartSet.t_particle[d_i].t_pose.f_x - f_gx; /*ローカル座標系へ変換*/
    t_local_pose.f_y = icSlam_st_PartSet.t_particle[d_i].t_pose.f_y - f_gy;

    icSlam_tagIntPoint t_ip = Ft_world2map (t_local_pose);
    d_flagKill = 0;

    if (Fd_isInside(t_ip)){
      if (pd_distmap[t_ip.d_x + t_ip.d_y*icSlam_st_mapSize.d_width] == ICSLAM_D_UNEXPLORED) {
        d_flagKill = 1;
      }
    } else {
      d_flagKill = 1;
      printf("[%s] kill ringout x:%d y:%d\n",__FUNCTION__, t_ip.d_x,t_ip.d_y);
    }

    if (d_flagKill == 1) {
      /* gFreeGridsがメモリを大量に使用するため、gFreeGridsを使用しない処理に変更   */
      /* RingOutしたパーティクルは自己位置に配置する                               */
      icSlam_st_PartSet.t_particle[d_i].t_pose.f_x = pt_estpose->f_x;
      icSlam_st_PartSet.t_particle[d_i].t_pose.f_y = pt_estpose->f_y;
      icSlam_st_PartSet.t_particle[d_i].t_pose.f_theta = pt_estpose->f_theta;

    }

  }

  return TRUE;
}

/**
 * @brief レーザデータを極座標からデカルト座標系へ変換する
 * @par
 * 座標系の変換を行うとともに、閾値よりも近い距離にあるレーザ点を間引く
 * 処理も行う。
 * 20110620 asahara 今後不使用
 */
int32_t icSlam_Fd_prepareLikelihood (const int32_t* pd_useLaser,
                                        icSlam_tagLaserData* pt_laser)
{
  icSlam_tagPoint t_lp;
  icSlam_tagPoint t_preLp;
  double  f_c           = 0.0;
  double  f_s           = 0.0;
  double  f_direction   = 0.0;
  double  f_beam        = 0.0;
  double  f_lpInterval  = 0.0;
  double  f_deltadelta = icSlam_st_mapSize.f_resolution*icSlam_st_mapSize.f_resolution;    //マップサイズ
  int32_t     d_i     = 0;
  int32_t     d_index = 0;

  assert(pd_useLaser!=(int32_t*)0);
  assert(pt_laser!=(icSlam_tagLaserData*)0);

  //初期化
  t_lp.f_x=0.0;
  t_lp.f_y=0.0;
  t_preLp.f_x=0.0;
  t_preLp.f_y=0.0;

  //水平URGを使用
  if (*pd_useLaser == 1) {
    for (d_i=0; d_i<ICSLAM_D_NUM_LASER; d_i++) {
      //レーザデータ(double)
      f_beam = pt_laser->f_data[d_i];
      //レーザデータが設定距離以内の場合に処理
      if (f_beam < icSlam_sf_maxLaserRange) {
        //レーザ1点の姿勢{(-90.0*M_PI/180.0)+カウンタ*(0.25*M_PI/180.0)}
        f_direction = ICSLAM_F_LASER_START_ANGLE + d_i*ICSLAM_F_LASER_RESOLUTION;
        //X方向用中間データ
        f_c = cos(f_direction);
        //Y方向用中間データ
        f_s = sin(f_direction);
        //レーザ原点からのオフセット(250.0mm)を考慮してX座標距離(f_beam * f_c)算出
        t_lp.f_x = f_beam * f_c + ICSLAM_F_LASER_OFFSET_X;
        //レーザ原点からのオフセット(0.0mm)を考慮してY座標距離(f_beam * f_s)算出
        t_lp.f_y = f_beam * f_s + ICSLAM_F_LASER_OFFSET_Y;
        f_lpInterval = (t_lp.f_x-t_preLp.f_x)*(t_lp.f_x-t_preLp.f_x)
                     + (t_lp.f_y-t_preLp.f_y)*(t_lp.f_y-t_preLp.f_y);

        if (f_lpInterval > f_deltadelta) {
          icSlam_st_rangeXY.t_laser[d_index] = t_lp;
          d_index = d_index+1;
          t_preLp = t_lp;
        }
      }
    }
  }
  icSlam_st_rangeXY.d_numLaser = d_index;  /*追加*/
  return TRUE;
}

/**
 * @brief  2次元ポイントクラウドをセットする
 * @par
 * 2次元ポイントクラウドをクライアント側で求めている場合、
 * icSlam_Fd_prepareLikelihoodの代わりにこちらの関数を利用する。
 */
int32_t icSlam_Fd_setRangePoints (const icSlam_tagRangeXY* pt_points)
{
  assert(pt_points!=(icSlam_tagRangeXY*)0);

  //コピーでいいのだろうか
  icSlam_st_rangeXY = *pt_points;

  return TRUE;
}

/**
 * @brief 重み（尤度）を求める
 * @param pd_distmap 地図データ
 * @param f_gx x軸の地図オフセット
 * @param f_gy y軸の地図オフセット
 */
double icSlam_Ff_likelihood (uint8_t* pd_distmap,
				const double f_gx,
				const double f_gy)
{
  int32_t                  d_i;
  icSlam_tagOrientedPoint t_local_pose; /* オフセットさせたパーティクル位置姿勢 */
  double               f_bestLikelihood = -DBL_MAX;
  double               f_ret_loglikelihood = (double)0.0;
  double               f_sum_loglikelihood = (double)0.0;
  double               f_reject_laser = (double)0.0;

  assert(pd_distmap!=(uint8_t*)0);

  for (d_i=0; d_i<icSlam_st_PartSet.d_numPart; d_i++) {
    /* グローバル座標系からローカル座標系へ変換 */
    t_local_pose.f_x = icSlam_st_PartSet.t_particle[d_i].t_pose.f_x - f_gx;
    t_local_pose.f_y = icSlam_st_PartSet.t_particle[d_i].t_pose.f_y - f_gy;
    t_local_pose.f_theta = icSlam_st_PartSet.t_particle[d_i].t_pose.f_theta;

    /* オフセットしたパーティクル位置姿勢を引数にする */
    f_ret_loglikelihood = Ff_logLikelihood (pd_distmap, t_local_pose, &f_reject_laser);
    icSlam_st_PartSet.t_particle[d_i].f_logWeight
      = icSlam_st_PartSet.t_particle[d_i].f_logWeight
      + f_ret_loglikelihood;

    icSlam_st_PartSet.t_particle[d_i].f_logWeight
      = icSlam_st_PartSet.t_particle[d_i].f_logWeight/2.0;

    if (icSlam_st_PartSet.t_particle[d_i].f_logWeight > f_bestLikelihood) {
      f_bestLikelihood = icSlam_st_PartSet.t_particle[d_i].f_logWeight;
    }
    
    f_sum_loglikelihood += f_ret_loglikelihood;

  }

  //return f_bestLikelihood;
  return f_sum_loglikelihood/icSlam_st_PartSet.d_numPart;
}

/**
 * @brief 重み（尤度）を一定の幅に収める
 */
int32_t icSlam_Fd_normalizeWeights (void)
{
  double f_minWeight;
  int32_t    d_ret;

  /*パーティクル数が0以下であれば異常終了*/
  assert(icSlam_st_PartSet.d_numPart>0);

  f_minWeight = 0.01/((double)(icSlam_st_PartSet.d_numPart));
  d_ret = Fd_normalizeWeights(f_minWeight);
  if (d_ret == FALSE) {
    /* error */
  }
  return TRUE;
}

/**
 * @brief Neff値が閾値を下回ったかどうかを判定
 * @param f_neff_threshold 判定閾値
 */
int32_t icSlam_Fd_checkNeff (double f_neff_threshold)
{
  int32_t d_ret;
  d_ret = Fd_computeNeff (f_neff_threshold);
  return d_ret;
}

/**
 * @brief リサンプリングを行う
 */
int32_t
icSlam_Fd_resample (void)
{
  int32_t                d_ret;
  int32_t                d_indexes[ICSLAM_D_MAX_NUM_PARTICLES];
  icSlam_tagParticleSet t_children;
  int32_t                d_indexSize;
  int32_t                d_i;

  d_indexSize = icSlam_st_PartSet.d_numPart;
  d_ret = Fd_resample (&d_indexes[0], d_indexSize);
  if(d_ret==FALSE) {
    /* error */
  }
  d_ret = Fd_repeatIndexes (&t_children, &icSlam_st_PartSet, d_indexes);
  if(d_ret==FALSE) {
    /* error */
  }
  for (d_i=0; d_i<d_indexSize; d_i++) {
    memcpy(&(icSlam_st_PartSet.t_particle[0]),
            &(t_children.t_particle[0]),
            d_indexSize*sizeof(icSlam_tagParticle));
  }
  return TRUE;
}

/**
 * @brief パーティクル集合から平均位置を抽出
 * @param pt_global_pose 平均位置格納用ポインタ．グローバル座標系とする.
 */
int32_t icSlam_Fd_getExpectedPose(icSlam_tagOrientedPoint* pt_global_pose)
{
  icSlam_tagOrientedPoint t_meanPose;
  icSlam_tagPoint         t_meanDirection;
  double               f_cweight = 0.0;
  int32_t                  d_i = 0;

  assert(pt_global_pose!=(icSlam_tagOrientedPoint*)0);

  t_meanPose.f_x = 0.0;
  t_meanPose.f_y = 0.0;
  t_meanPose.f_theta = 0.0;
  t_meanDirection.f_x = 0.0;
  t_meanDirection.f_y = 0.0;

  /*単位ベクトル×尤度に基づいて平均位置姿勢を算出*/
  for (d_i=0; d_i<icSlam_st_PartSet.d_numPart; d_i++) {
    t_meanPose.f_x
      = t_meanPose.f_x
      + (icSlam_st_PartSet.t_particle[d_i].t_pose.f_x
         * icSlam_st_PartSet.t_particle[d_i].f_weight);
    t_meanPose.f_y
      = t_meanPose.f_y
      + (icSlam_st_PartSet.t_particle[d_i].t_pose.f_y
         * icSlam_st_PartSet.t_particle[d_i].f_weight);
    t_meanDirection.f_x
      = t_meanDirection.f_x
      + (cos(icSlam_st_PartSet.t_particle[d_i].t_pose.f_theta)
         * icSlam_st_PartSet.t_particle[d_i].f_weight);
    t_meanDirection.f_y
      = t_meanDirection.f_y
      + (sin(icSlam_st_PartSet.t_particle[d_i].t_pose.f_theta)
         * icSlam_st_PartSet.t_particle[d_i].f_weight);
    f_cweight
      = f_cweight + icSlam_st_PartSet.t_particle[d_i].f_weight;
  }

  // 同assertで停止しないようにすると、自己位置がnanになる事象が発生します.
  assert (f_cweight > DBL_MIN);

  pt_global_pose->f_x = t_meanPose.f_x / f_cweight;
  pt_global_pose->f_y = t_meanPose.f_y / f_cweight;
  pt_global_pose->f_theta = atan2(t_meanDirection.f_y, t_meanDirection.f_x);

  return TRUE;
}

/**
 * @brief 地図パラメータをセット
 * @param d_width 地図の横幅のグリッド数
 * @param d_height 地図の縦幅のグリッド数
 * @param f_resolution １グリッドの分解能(m)
 * @param f_threshold 地図のポテンシャルの幅(m)
 * @param f_filtering 地図にない障害物を消すための閾値(m)　壁から閾値m離れたら消す
 */
int32_t icSlam_Fd_setMapSize (int32_t d_width,
                                 int32_t d_height,
                                 double f_resolution,
			     double f_threshold,
			     double f_filtering)
{
  icSlam_st_mapSize.d_width      = d_width;
  icSlam_st_mapSize.d_height     = d_height;
  icSlam_st_mapSize.f_resolution = f_resolution;
  icSlam_st_mapSize.f_threshold  = f_threshold;
  icSlam_st_intMapOffset.d_x     = (int32_t)icSlam_st_mapSize.d_width/2;
  icSlam_st_intMapOffset.d_y     = (int32_t)icSlam_st_mapSize.d_height/2;
  icSlam_sf_filtering            = f_filtering;

  return TRUE;
}

/**
 * @brief パーティクルを初期位置にセットする
 * @param t_initPose パーティクルの初期位置
 */
int32_t icSlam_Fd_setInitParticles(icSlam_tagOrientedPoint t_initPose,
                                     int32_t d_numParticles,
                                     double f_standard_deviation_xy,
                                     double f_standard_deviation_theta)
{
  int32_t d_i = 0;

  assert(d_numParticles>0); //パーティクル数が0以下であれば終了

  if (d_numParticles > ICSLAM_D_NUM_TRACKING_PARTICLES) {
    fprintf(stderr,
            "number of particles(%d) is more than maximum(%d)\n",
            d_numParticles,
            ICSLAM_D_NUM_TRACKING_PARTICLES);
    return FALSE;
  }

  for (d_i=0; d_i<d_numParticles; d_i++) {
    icSlam_st_PartSet.t_particle[d_i].t_pose.f_x
      = t_initPose.f_x + Ff_sampleGaussian (f_standard_deviation_xy);
    icSlam_st_PartSet.t_particle[d_i].t_pose.f_y
      = t_initPose.f_y + Ff_sampleGaussian (f_standard_deviation_xy);
    icSlam_st_PartSet.t_particle[d_i].t_pose.f_theta
      = t_initPose.f_theta + Ff_sampleGaussian (f_standard_deviation_theta);
    icSlam_st_PartSet.t_particle[d_i].f_logWeight = 0.0;
    icSlam_st_PartSet.t_particle[d_i].f_weight = 1.0;
  }
  icSlam_st_PartSet.d_numPart = d_numParticles;


  return TRUE;
}

/**
 * @brief 運動モデルのノイズパラメータをセットする
 * @param f_noiseSxy
 */
int32_t icSlam_Fd_setMotionNoise (double f_noiseSxy,
                                    double f_noiseStr,
                                    double f_noiseSrr,
                                    double f_noiseStt,
                                    double f_noiseSrt)
{
  icSlam_sf_sxy = f_noiseSxy;
  icSlam_sf_str = f_noiseStr;
  icSlam_sf_srr = f_noiseSrr;
  icSlam_sf_stt = f_noiseStt;
  icSlam_sf_srt = f_noiseSrt;

  return 0;
}

/**
 * @brief 重み（尤度）を求める
 * @param t_local_pose ローカル座標系での位置
 */
static double Ff_logLikelihood(const uint8_t* const pd_distmap,
				  const icSlam_tagOrientedPoint t_local_pose,
				  double* f_reject_laser)
{
  double f_c              = cos(t_local_pose.f_theta);
  double f_s              = sin(t_local_pose.f_theta);
  double f_likelihood     = 0.0;
  double f_sqrtLikelihood = 0.0;
  icSlam_tagPoint    t_beam_point;
  icSlam_tagIntPoint t_int_beam_point;
  int32_t             d_i;
  int32_t             cnt_allowable = 0; 

  assert(pd_distmap != (uint8_t*)0);

  for (d_i=0; d_i<icSlam_st_rangeXY.d_numLaser; d_i++) {
    f_sqrtLikelihood = 0.0;

    /* レーザのエンドポイントをロボット位置姿勢を使ってグローバルに変換 */
    t_beam_point.f_x = t_local_pose.f_x
                     + icSlam_st_rangeXY.t_laser[d_i].f_x*f_c
                     - icSlam_st_rangeXY.t_laser[d_i].f_y*f_s;
    t_beam_point.f_y = t_local_pose.f_y
                     + icSlam_st_rangeXY.t_laser[d_i].f_x*f_s
                     + icSlam_st_rangeXY.t_laser[d_i].f_y*f_c;

    /* グローバル座標から地図座標へ変換 */
    t_int_beam_point = Ft_world2map(t_beam_point);

    if (Fd_isInside(t_int_beam_point)) { /* レーザエンドポイントが地図に収まっている場合 */
      if (pd_distmap[t_int_beam_point.d_x + t_int_beam_point.d_y*icSlam_st_mapSize.d_width]==ICSLAM_D_UNEXPLORED) {
        f_sqrtLikelihood = icSlam_st_mapSize.f_threshold;
      } else {
        f_sqrtLikelihood = (double)((uint8_t)255
                         - pd_distmap[t_int_beam_point.d_x + t_int_beam_point.d_y*icSlam_st_mapSize.d_width]); /*壁からの誤差へ変換*/
        f_sqrtLikelihood = f_sqrtLikelihood/255.0; /*スケール変換*/
        f_sqrtLikelihood = (f_sqrtLikelihood*icSlam_st_mapSize.f_threshold);

      }
    } else { /* レーザエンドポイントが地図からはみ出た場合 */
      f_sqrtLikelihood = icSlam_st_mapSize.f_threshold;
    }

    if(f_sqrtLikelihood*f_sqrtLikelihood < icSlam_sf_filtering * icSlam_sf_filtering){
      
      f_likelihood -= f_sqrtLikelihood * f_sqrtLikelihood; /* [m^2] */
      cnt_allowable++;
    }
    else{
      //do nothing
    }
  }

  if (icSlam_st_rangeXY.d_numLaser==0 || cnt_allowable==0 ) {
    return 0.0;
  }


  *f_reject_laser = (double)(icSlam_st_rangeXY.d_numLaser - cnt_allowable);

  // fprintf(stderr,"allowable: %d\n", cnt_allowable);

  return f_likelihood/(double)(cnt_allowable);
}

/**
 * @brief Neff値を下回っているかどうかチェックする
 * @param f_neff_threshold Neff閾値
 */
static int32_t Fd_computeNeff(const double f_neff_threshold)
{
  double f_sum = 0.0;
  double f_cum = 0.0;
  int32_t    d_i = 0;

  for (d_i=0; d_i<icSlam_st_PartSet.d_numPart; d_i++) {
    f_sum = f_sum + icSlam_st_PartSet.t_particle[d_i].f_weight;
  }

  if ((!(fabs(f_sum - 0.0) > (DBL_EPSILON * f_sum)))
      || (f_sum < 0.0)) {
    return FALSE;
  }

  for (d_i=0; d_i<icSlam_st_PartSet.d_numPart; d_i++) {
    f_cum = f_cum
      + (icSlam_st_PartSet.t_particle[d_i].f_weight/f_sum)
      * (icSlam_st_PartSet.t_particle[d_i].f_weight/f_sum);
  }

  if ((!(fabs(f_cum - 0.0) > (DBL_EPSILON * f_cum))) || (f_cum < 0.0)) {
    return FALSE;
  }
  icSlam_sf_neff = 1.0/f_cum;

  if ((!(fabs((f_neff_threshold*icSlam_st_PartSet.d_numPart) - icSlam_sf_neff)
         > (DBL_EPSILON * (f_neff_threshold*icSlam_st_PartSet.d_numPart))))
      || ((f_neff_threshold*icSlam_st_PartSet.d_numPart) < icSlam_sf_neff)) {
    return FALSE;
  } else {
    return TRUE;
  }
}

/**
 * @brief 現在のNeff値を取得する
 * @return Neff値
 */
double icSlam_Ff_getNeff()
{
  return icSlam_sf_neff;
}

/**
 * @brief 移動量＋ガウシアンノイズの分だけ移動させる
 * @param t_p 現在位置姿勢
 * @param t_dPose 移動量
 * @return 移動後の位置姿勢
 */
static icSlam_tagOrientedPoint Ft_drawMotion(const icSlam_tagOrientedPoint t_p,
                                                const icSlam_tagOrientedPoint t_dPose)
{
  icSlam_tagOrientedPoint t_noisyDPose = t_dPose;
  icSlam_tagOrientedPoint t_retPose;
  double f_nx     = 0.0;
  double f_ny     = 0.0;
  double f_ntheta = 0.0;

  double f_fabsDx = fabs(t_dPose.f_x);
  double f_fabsDy = fabs(t_dPose.f_y);
  double f_fabsDtheta = fabs(t_dPose.f_theta);

  f_nx = icSlam_sf_stt*f_fabsDx + icSlam_sf_srt*f_fabsDtheta + icSlam_sf_sxy*f_fabsDy;
  f_ny = icSlam_sf_stt*f_fabsDy + icSlam_sf_srt*f_fabsDtheta + icSlam_sf_sxy*f_fabsDx;
  f_ntheta = icSlam_sf_srr*f_fabsDtheta
           + icSlam_sf_str*sqrt(f_fabsDx*f_fabsDx + f_fabsDy*f_fabsDy);

  t_noisyDPose.f_x = t_noisyDPose.f_x + Ff_sampleGaussian (f_nx);
  t_noisyDPose.f_y = t_noisyDPose.f_y + Ff_sampleGaussian (f_ny);
  t_noisyDPose.f_theta = t_noisyDPose.f_theta + Ff_sampleGaussian (f_ntheta);
  t_noisyDPose.f_theta = Ff_pmPI (t_noisyDPose.f_theta);

  t_retPose = Ft_absoluteSum (t_p, t_noisyDPose);

  return t_retPose;
}

/**
 * @brief 移動量の足し算を行う
 */
static icSlam_tagOrientedPoint Ft_absoluteSum (const icSlam_tagOrientedPoint t_p1,
                                                  const icSlam_tagOrientedPoint t_p2)
{
  icSlam_tagOrientedPoint t_retp;
  double f_s = sin(t_p1.f_theta);
  double f_c = cos(t_p1.f_theta);

  t_retp.f_x = t_p1.f_x + f_c*t_p2.f_x - f_s*t_p2.f_y;
  t_retp.f_y = t_p1.f_y + f_s*t_p2.f_x + f_c*t_p2.f_y;
  t_retp.f_theta = t_p1.f_theta + t_p2.f_theta;

  t_retp.f_theta = Ff_pmPI (t_retp.f_theta);

  return t_retp;
}

/**
 * @brief 中心極限定理を利用した正規乱数発生関数
 * @param f_sd 標準偏差
 * @return 正規乱数
 */
static double Ff_sampleGaussian (const double f_sd)
{
  double f_sumRand = 0.0;
  double f_ret = 0.0;
  int32_t    d_i = 0;

  // 12, 2.0, 6.0といった数字はアルゴリズムで決められているのでそのまま利用する
  for (d_i=0; d_i<12; d_i++) {
    f_sumRand += (2.0*((double)rand()/(double)RAND_MAX) - 1.0);
  }
  f_ret = ((f_sd/6.0)*f_sumRand);

  return f_ret;
}

/**
 * @brief 重み（尤度）を一定の幅内に収める
 * @param f_min_weight 最小尤度
 */
static int32_t Fd_normalizeWeights(const double f_min_weight)
{
  double f_minLogWeight = DBL_MAX;
  double f_maxLogWeight = -DBL_MAX;
  int32_t d_i;

  for (d_i=0; d_i<icSlam_st_PartSet.d_numPart; d_i++) {
    if(icSlam_st_PartSet.t_particle[d_i].f_logWeight < f_minLogWeight) {
      f_minLogWeight = icSlam_st_PartSet.t_particle[d_i].f_logWeight;
    }
    if(icSlam_st_PartSet.t_particle[d_i].f_logWeight > f_maxLogWeight) {
      f_maxLogWeight = icSlam_st_PartSet.t_particle[d_i].f_logWeight;
    }
  }

  double f_minLogNormalizedValue = log(f_min_weight);
  double f_maxLogNormalizedValue = log(1.0);
  double f_dn = f_maxLogNormalizedValue - f_minLogNormalizedValue;
  double f_dw = f_maxLogWeight - f_minLogWeight;
  if((!( fabs(f_dw - 0.00000001) > (DBL_EPSILON * f_dw))) || (f_dw < 0.00000001)) {
    f_dw = 1.0;
  }

  double f_scale = f_dn/f_dw;
  double f_offset = -f_maxLogWeight*f_scale;

  for (d_i=0; d_i<icSlam_st_PartSet.d_numPart; d_i++) {
    double f_w = icSlam_st_PartSet.t_particle[d_i].f_logWeight;
    f_w = f_scale*f_w + f_offset;
    icSlam_st_PartSet.t_particle[d_i].f_weight = exp(f_w);
  }

  return TRUE;
}

/**
 * @brief リサンプリングを行う
 * @param pd_indexes 選択された親パーティクルのindexを表す配列のポインタ
 * @param d_size パーティクル個数
 */
static int32_t Fd_resample (int32_t* const pd_indexes, const int32_t d_size)
{
  double f_cweight = 0.0;
  int32_t    d_i;

  assert(pd_indexes!=(int32_t*)0);

  /*パーティクル数エラーチェック*/
  assert(d_size==icSlam_st_PartSet.d_numPart);

  /* compute the cumulative weights */
  for (d_i=0; d_i<icSlam_st_PartSet.d_numPart; d_i++) {
    f_cweight = f_cweight + icSlam_st_PartSet.t_particle[d_i].f_weight;
  }

  if (icSlam_st_PartSet.d_numPart <= 0) {
    return FALSE;
  }

  /* compute the interval */
  double f_interval = f_cweight/icSlam_st_PartSet.d_numPart;

  /* compute the initial target weight */
  double f_target = f_interval * ((double)(rand())/(double)(RAND_MAX));

  /* compute the resampled indexes */
  f_cweight = 0.0;

  int32_t d_n = 0;
  for (d_i=0; d_i<icSlam_st_PartSet.d_numPart; d_i++) {
    f_cweight = f_cweight + icSlam_st_PartSet.t_particle[d_i].f_weight;
    while (f_cweight > f_target) {
      pd_indexes[d_n] = d_i;
      d_n++;
      f_target = f_target + f_interval;
    }
  }

  return TRUE;
}
/**
 * @brief 親パーティクルから子パーティクルを再生する
 * どの親から生まれるかは引数で指定してやる必要がある
 * @param pt_children 子パーティクルセット
 * @param pt_parents 親パーティクルセット
 * @param pd_indexes 子が生成される親パーティクルのindex番号を表す配列のポインタ
 */
static int32_t Fd_repeatIndexes (icSlam_tagParticleSet* const pt_children,
                                   const icSlam_tagParticleSet* const pt_parents,
                                   const int32_t* const pd_indexes)
{
  int32_t d_i;

  /*ポインタがNULLの時は異常終了*/
  assert(pt_children!=(icSlam_tagParticleSet*)0);
  assert(pt_parents!=(icSlam_tagParticleSet*)0);
  assert(pd_indexes!=(int32_t*)0);

  for (d_i=0; d_i<pt_parents->d_numPart; d_i++) {
    pt_children->t_particle[d_i] = pt_parents->t_particle[pd_indexes[d_i]];
  }

  return TRUE;
}

/**
 * @brief 位置が地図からはみ出ていないかどうかをチェック
 */
static int32_t Fd_isInside (const icSlam_tagIntPoint t_int_point)
{
  if ((t_int_point.d_x < (int32_t)icSlam_st_mapSize.d_width)
      && (0 <= t_int_point.d_x)
      && (t_int_point.d_y < (int32_t)icSlam_st_mapSize.d_height)
      && (0 <= t_int_point.d_y)) {
    return TRUE;
  } else {
    return FALSE;
  }
}

/**
 * @brief 角度を±πに変換
 */
static double Ff_pmPI (const double f_theta)
{
  double f_retTheta;

  f_retTheta = fmod(f_theta, 2*M_PI);
  if (f_retTheta>M_PI) {
    f_retTheta = f_retTheta - 2*M_PI;
  }

  if((!( fabs(f_retTheta - (-M_PI)) > (DBL_EPSILON * f_retTheta))) || (f_retTheta < (-M_PI))) {
    f_retTheta = f_retTheta + 2*M_PI;
  }

  return f_retTheta;
}

/**
 * @brief 現在のパーティクル内容を取得
 * @note 参照専用とすること
 */
int32_t icSlam_Fd_GetParticlePositions(icSlam_tagParticleSet* pt_particle)
{
  memcpy(&(pt_particle->t_particle[0]),
          &(icSlam_st_PartSet.t_particle[0]),
          icSlam_st_PartSet.d_numPart * sizeof(icSlam_tagParticle));
  pt_particle->d_numPart = icSlam_st_PartSet.d_numPart;
  return ICSLAM_D_RET_SUCCESS;
}

