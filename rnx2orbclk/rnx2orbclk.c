/*------------------------------------------------------------------------------
* rnx2rtkp.c : read rinex obs/nav files and compute receiver positions
*
*          Copyright (C) 2007-2009 by T.TAKASU, All rights reserved.
*
* version : $Revision: 1.1 $ $Date: 2008/07/17 21:55:16 $
* history : 2007/01/16  1.0 new
*           2007/03/15  1.1 add library mode
*           2007/05/08  1.2 separate from postpos.c
*           2009/01/20  1.3 support rtklib 2.2.0 api
*           2009/12/12  1.4 support glonass
*                           add option -h, -a, -l, -x
*           2010/01/28  1.5 add option -k
*           2010/08/12  1.6 add option -y implementation (2.4.0_p1)
*           2014/01/27  1.7 fix bug on default output time format
*-----------------------------------------------------------------------------*/
#include <stdarg.h>
#include "rtklib.h"

static const char rcsid[]="$Id: rnx2rtkp.c,v 1.1 2008/07/17 21:55:16 ttaka Exp $";

#define PROGNAME    "rnx2rtkp"          /* program name */
#define MAXFILE     8                   /* max number of input files */
#define SQR(x)      ((x)*(x))
/* help text -----------------------------------------------------------------*/
static const char *help[]={
"",
" usage: rnx2rtkp [option]... file file [...]",
"",
" Read RINEX OBS/NAV/GNAV/HNAV/CLK, SP3, SBAS message log files and ccompute ",
" receiver (rover) positions and output position solutions.",
" The first RINEX OBS file shall contain receiver (rover) observations. For the",
" relative mode, the second RINEX OBS file shall contain reference",
" (base station) receiver observations. At least one RINEX NAV/GNAV/HNAV",
" file shall be included in input files. To use SP3 precise ephemeris, specify",
" the path in the files. The extension of the SP3 file shall be .sp3 or .eph.",
" All of the input file paths can include wild-cards (*). To avoid command",
" line deployment of wild-cards, use \"...\" for paths with wild-cards.",
" Command line options are as follows ([]:default). With -k option, the",
" processing options are input from the configuration file. In this case,",
" command line options precede options in the configuration file.",
"",
" -?        print help",
" -k file   input options from configuration file [off]",
" -o file   set output file [stdout]",
" -ts ds ts start day/time (ds=y/m/d ts=h:m:s) [obs start time]",
" -te de te end day/time   (de=y/m/d te=h:m:s) [obs end time]",
" -ti tint  time interval (sec) [all]",
" -p mode   mode (0:single,1:dgps,2:kinematic,3:static,4:moving-base,",
"                 5:fixed,6:ppp-kinematic,7:ppp-static) [2]",
" -m mask   elevation mask angle (deg) [15]",
" -f freq   number of frequencies for relative mode (1:L1,2:L1+L2,3:L1+L2+L5) [2]",
" -v thres  validation threshold for integer ambiguity (0.0:no AR) [3.0]",
" -b        backward solutions [off]",
" -c        forward/backward combined solutions [off]",
" -i        instantaneous integer ambiguity resolution [off]",
" -h        fix and hold for integer ambiguity resolution [off]",
" -e        output x/y/z-ecef position [latitude/longitude/height]",
" -a        output e/n/u-baseline [latitude/longitude/height]",
" -n        output NMEA-0183 GGA sentence [off]",
" -g        output latitude/longitude in the form of ddd mm ss.ss' [ddd.ddd]",
" -t        output time in the form of yyyy/mm/dd hh:mm:ss.ss [sssss.ss]",
" -u        output time in utc [gpst]",
" -d col    number of decimals in time [3]",
" -s sep    field separator [' ']",
" -r x y z  reference (base) receiver ecef pos (m) [average of single pos]",
" -l lat lon hgt reference (base) receiver latitude/longitude/height (deg/m)",
" -y level  output soltion status (0:off,1:states,2:residuals) [0]",
" -x level  debug trace level (0:off) [0]"
};
/* show message --------------------------------------------------------------*/
extern int showmsg(char *format, ...)
{
    va_list arg;
    va_start(arg,format); vfprintf(stderr,format,arg); va_end(arg);
    fprintf(stderr,"\r");
    return 0;
}
extern void settspan(gtime_t ts, gtime_t te) {}
extern void settime(gtime_t time) {}

/* print help ----------------------------------------------------------------*/
static void printhelp(void)
{
    int i;
    for (i=0;i<(int)(sizeof(help)/sizeof(*help));i++) fprintf(stderr,"%s\n",help[i]);
    exit(0);
}
/* rnx2rtkp main -------------------------------------------------------------*/
int main(int argc, char **argv)
{
    prcopt_t prcopt=prcopt_default;
    prcopt.mode  =PMODE_KINEMA;
    // prcopt.navsys=SYS_GPS|SYS_GLO|SYS_GAL;
    prcopt.navsys=SYS_GPS;
    prcopt.refpos=1;
    prcopt.glomodear=1;
    prcopt.sateph = 1; // 0 for broadcast ephemeris; 1 for precise ephemeris
    prcopt.nf = 2;

    
    nav_t navs={0};         /* navigation data */
    pcvs_t pcvss={0};        /* receiver antenna parameters */
    pcvs_t pcvsr={0};        /* satellite antenna parameters */
    sta_t stas[MAXRCV];      /* station infomation */
    nav_t *nav = &navs; 
    nav->ne=nav->nemax=0;
    nav->nc=nav->ncmax=0;
    nav->eph =NULL; nav->n =nav->nmax =0;
    prcopt_t *popt = &prcopt;

    
    gtime_t s_time;  // time of start
    int year=2016;
    char* yy="16";
    char * station = "com";
    s_time.time = 1451606400; // 2016-01-01
    s_time.sec = 0;

    // int year=2009;
    // char* yy="09";
    // char * station = "igs";
    // s_time.time = 1230768000; // 2009-01-01
    // s_time.sec = 0;

    int total_day=2;
    
    /*read all related files*/
    FILE *fp_sp3, *fp;
    for(int doy_buff=1;doy_buff<=total_day;doy_buff++){
        gtime_t inq_time_buff=timeadd(s_time,(doy_buff-1)*86400);// time of first inquire in current doy
        double ep[6];
        time2epoch(inq_time_buff,ep);
        int year_buff=(int)ep[0];
        int week_buff;
        double tow_buff = time2gpst(inq_time_buff,&week_buff);
        int dow_buff=(int)floor(tow_buff/86400.0);

        /* read precise ephemeris files*/
        char sp3file[40];
        sprintf(sp3file, "./pce_%d/%s%d%d.sp3", year,station,week_buff,dow_buff);
        fp_sp3=fopen(sp3file,"r");
        gtime_t time_tmp={0};
        double bfact[2]={0};
        int ns,sats_tmp[MAXSAT]={0};
        char type_sp3=' ',tsys_sp3[4]="";
        /* read sp3 header */
        ns=readsp3h(fp_sp3,&time_tmp,&type_sp3,sats_tmp,bfact,tsys_sp3);
        /* read sp3 body */
        readsp3b(fp_sp3,type_sp3,sats_tmp,ns,bfact,tsys_sp3,doy_buff,0,nav);
        fclose(fp_sp3);

        /* read navigation data */
        double ver;
        int sys,tsys;
        char tobs[6][MAXOBSTYPE][4]={{""}};
        char type=' ';
        sta_t *sta;
        char rnxfile[40];
        if(doy_buff<10){sprintf(rnxfile, "./brd_igs_%d/brdc00%d0.%sn", year,doy_buff,yy);}
        else if(doy_buff<100){sprintf(rnxfile, "./brd_igs_%d/brdc0%d0.%sn",year, doy_buff,yy);}
        else{sprintf(rnxfile, "./brd_igs_%d/brdc%d0.%sn",year, doy_buff,yy);}
        fp=fopen(rnxfile,"r");
        readrnxh(fp,&ver,&type,&sys,&tsys,tobs,nav,stas);//read rinex header
        readrnxnav(fp,popt->rnxopt[0],ver,sys,nav);//read rinex body
        fclose(fp);
    }
    uniqnav(nav); //delete duplicated ephemeris

    /* set antenna paramters for precise product*/
    setpcv(s_time,&prcopt,&navs,&pcvss,&pcvsr,stas);

    /* set antenna paramters for broadcast product according to NGA document*/
    for (int i=0;i<32;i++) {
        int satno = i+1;
        double x,y,z;
        switch (satno){
            case 1: x=0.3910; y=0.0000; z=1.0910;break;
            case 2: x=-0.0099; y=0.0061; z=-0.0820;break;
            case 3: x=0.3950; y=0.0003; z=1.0907;break;
            case 4: x=0.003785; y=-0.018085; z=1.2324;break;
            case 5: x=0.0029; y=-0.0001; z=-0.0167;break;
            case 6: x=0.3947; y=-0.0010; z=1.0917;break;
            case 7: x=0.0013; y=0.0003; z=0.0006;break;
            case 8: x=0.3962; y=-0.0003; z=1.0856;break;
            case 9: x=0.3955; y=-0.0020; z=1.0922;break;
            case 10: x=0.3962; y=-0.0013; z=1.0831;break;
            case 11: x=0.0019; y=0.0011; z=1.5141;break;
            case 12: x=-0.0102; y=0.0059; z=-0.0936;break;
            case 13: x=0.0024; y=0.0025; z=1.6140;break;
            case 14: x=0.0018; y=0.0002; z=1.6137;break;
            case 15: x=-0.0100; y=0.0058; z=-0.0123;break;
            case 16: x=-0.0098; y=0.0060; z=1.6630;break;
            case 17: x=-0.0100; y=0.0060; z=-0.1008;break;
            case 18: x=0.2794; y=0.0000; z=0.9519;break;
            case 19: x=-0.0079; y=0.0046; z=-0.0180;break;
            case 20: x=0.0022; y=0.0014; z=1.6140;break;
            case 21: x=0.0023; y=-0.0006; z=1.5840;break;
            case 22: x=0.0018; y=-0.0009; z=0.0598;break;
            case 23: x=-0.0088; y=0.0035; z=0.0004;break;
            case 24: x=0.3920; y=0.0020; z=1.0930;break;
            case 25: x=0.3920; y=0.0020; z=1.0930;break;
            case 26: x=0.3949; y=-0.0011; z=1.0927;break;
            case 27: x=0.3914; y=0.0003; z=1.0904;break;
            case 28: x=0.0018; y=0.0007; z=1.5131;break;
            case 29: x=-0.0101; y=0.0059; z=-0.0151;break;
            case 30: x=0.3952; y=-0.00080; z=1.0904;break;
            case 31: x=0.0016; y=0.0003; z=-0.0575;break;
            case 32: x=0.3966; y=-0.00020; z=1.0843;break;
        }
        pcv_t pcv; 
        pcv=nav->pcvs[i];
        pcv_t *ppcv = &pcv;
        ppcv->off[0][0]=x;ppcv->off[0][1]=y;ppcv->off[0][2]=z;
        ppcv->off[1][0]=x;ppcv->off[1][1]=y;ppcv->off[1][2]=z;
        nav->pcvsb[i]=*ppcv;
    }

    /* inquire */
    FILE* log_file = fopen("./outpos_all.csv", "w"); // Open in "append" mode
    // fprintf(log_file, "Year,Doy,SoD,PRN,Xp,Yp,Zp,dtp,Xb,Yb,Zb,dtb,clkDiff\n");
    fprintf(log_file, "Year,Doy,SoD,PRN,Rp,Ap,Cp,dtp,Rb,Ab,Cb,dtb,radDiff,atDiff,ctDiff,clkDiff\n");
    for(int doy=1;doy<=1;doy++){
        gtime_t inq_time_s=timeadd(s_time,(doy-1)*86400);// time of first inquire in current doy
        double ep[6];
        time2epoch(inq_time_s,ep);
        int year=(int)ep[0];
        double step = 1; //seconds
        for(int i_epoch=0;i_epoch<86400/step;i_epoch++){
            int flag=0;
            if(doy==1 && step*i_epoch==82800){
                flag=1;
            }
            if(doy==3 && step*i_epoch==85800){
                flag=1;
            }
            if(doy==6 && step*i_epoch==65100){
                flag=1;
            }
            if(doy==5 && step*i_epoch==900){
                flag=1;
            }


            gtime_t inq_time=timeadd(inq_time_s,step*i_epoch);// inquire time
            char s_inq[32];
            time2str(inq_time, s_inq,1);
            printf("Inquire time : %s\n",s_inq);
            gtime_t tut0;
            int sod=time2sec(inq_time,&tut0);
            
            for(int sat=1;sat<=32;sat++){ //inquire satellite number
                int n_inq = 1; // number of inquires (each inquire is to solve the precise position and clock of a given satellite at given time)
                int opt = 1; //sat postion option (0: center of mass, 1: antenna phase center)
                /* precise satellite position and clock */
                double *rs_pce,*dts_pce,*var_pce;
                rs_pce=mat(6,n_inq); dts_pce=mat(2,n_inq); var_pce=mat(1,n_inq);
                if(!peph2pos(inq_time,sat,nav,opt,rs_pce,dts_pce,var_pce)){
                    continue;
                }
                /* precise position transformation: ECEF->RAC */
                double pos_pce_rac[3];
                ecef2rac(rs_pce,rs_pce,pos_pce_rac);
                
                if(!opt){
                /* precise clock transformation: APC->CoM */
                    const double *lam_tmp=nav->lam[sat-1];
                    const pcv_t *pcv_tmp =nav->pcvs+sat-1;
                    int f1=0,f2=1;
                    double gamma,C1,C2;
                    gamma=SQR(lam_tmp[f2])/SQR(lam_tmp[f1]);
                    C1=gamma/(gamma-1.0);
                    C2=-1.0 /(gamma-1.0);
                    double apc_LC_z = C1*pcv_tmp->off[f1][2]+C2*pcv_tmp->off[f2][2]; // z-direction APC offset in the ionosphere-free combination in satellite body frame
                    dts_pce[0] = dts_pce[0] + apc_LC_z/CLIGHT; // APC -> MoC 
                }   
                
                /* broadcast satellite position and clock */
                double *rs_bce,*dts_bce,*var_bce;
                int svh_bce[n_inq];
                rs_bce=mat(6,n_inq); dts_bce=mat(2,n_inq); var_bce=mat(1,n_inq); 
                if(!ephpos(inq_time,inq_time,sat,nav,-1,rs_bce,dts_bce,var_bce,svh_bce)){
                    continue;
                }

                if(!opt){
                /* broadcast position transformation: APC->CoM */
                    double dant_bce[3]={0}; // APC offeset in ECEF
                    satantoff_bce(inq_time,rs_bce,sat,nav,dant_bce); 
                    for (int ii=0;ii<3;ii++) {
                        rs_bce[ii]=rs_bce[ii] - dant_bce[ii]; // negative sign: APC->CoM
                    }
                }
                /* broadcast position transformation: ECEF->RAC */
                double pos_bce_rac[3];
                ecef2rac(rs_pce,rs_bce,pos_bce_rac);
                
                if(!opt){
                /* broadcast clock transformation: APC->CoM */
                    const double *lam_tmp=nav->lam[sat-1];
                    int f1=0,f2=1;
                    double gamma,C1,C2;
                    gamma=SQR(lam_tmp[f2])/SQR(lam_tmp[f1]);
                    C1=gamma/(gamma-1.0);
                    C2=-1.0 /(gamma-1.0);
                    const pcv_t *pcv_tmp_bce =nav->pcvsb+sat-1;
                    double apc_LC_z_bce = C1*pcv_tmp_bce->off[f1][2]+C2*pcv_tmp_bce->off[f2][2]; // z-direction APC offset in the ionosphere-free combination in satellite body frame
                    dts_bce[0] = dts_bce[0] + apc_LC_z_bce/CLIGHT; // APC -> MoC 
                }

                fprintf(log_file, "%d,%d,%d,%d,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                year,doy,sod,sat, 
                // rs_pce[0],rs_pce[1],rs_pce[2],rs_bce[0],rs_bce[1],rs_bce[2], // ECEF
                pos_pce_rac[0],pos_pce_rac[1],pos_pce_rac[2],CLIGHT*dts_pce[0], // RAC precise orbit & clock
                pos_bce_rac[0],pos_bce_rac[1],pos_bce_rac[2],CLIGHT*dts_bce[0], // RAC broadcast orbit & clock
                pos_pce_rac[0]-pos_bce_rac[0], // radDiff
                pos_pce_rac[1]-pos_bce_rac[1], // atDiff
                pos_pce_rac[2]-pos_bce_rac[2], // ctDiff
                CLIGHT*(dts_pce[0]-dts_bce[0])); // clkDiff
                
                int aa;
                aa=0;
            }
        }
    }
    // Close the log file
    fclose(log_file);
    return 0;
}
