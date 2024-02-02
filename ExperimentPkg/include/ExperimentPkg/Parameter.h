#ifndef __PARAMETER_H
#define __PARAMETER_H
#include "math.h"

//Definition of all parameters


//Excavator parameters//
#define L1 3710.347065261762
#define L2 1615.225037542078
#define PI 3.1415926
#define alphaMin double(-49.5963*3.1415926 / 180)
#define alphaMax double(76.9360*3.1415926 / 180)
#define beitaMin double(-156.5645*3.1415926 / 180)
#define beitaMax double(-3.7470*3.1415926 / 180)
#define gamaMin double(-122*3.1415926 / 180)
#define gamaMax double(47.0535*3.1415926 / 180)


//System parameters obtained by PMMM+LM method
#define lOM1_PMMMLM 3512.612984252831
#define lFM2_PMMMLM 797.9969081696979
#define lQM3_PMMMLM 338.4189804282952
#define aerfa1_PMMMLM 0.029205010067881
#define beita1_PMMMLM 0.067461552859301
#define gama1_PMMMLM 0.923079505283805
#define rou1_PMMMLM -1243.327847461868
#define rou2_PMMMLM 1144.550779707609
#define rou3_PMMMLM 588.1271998672107
#define fai1_PMMMLM -0.649188085054620
#define fai2_PMMMLM -0.744719710705827
#define fai3_PMMMLM -1.490349485444234
#define Z1_PMMMLM -50.200356240752080
#define Z2_PMMMLM 7.775110634868038
#define Z3_PMMMLM -193.0843631245339
#define k1_PMMMLM -0.095268563927571
#define k2_PMMMLM 0.122688103605218
#define p1_PMMMLM -0.003819259007449
#define p2_PMMMLM -0.005837406288166
#define k3_PMMMLM -0.077136430937939
#define fx_PMMMLM 847.2160236292539
#define u0_PMMMLM 632.8224264614696
#define fy_PMMMLM 898.2635039711116
#define v0_PMMMLM 559.5711079617985

//The system parameters obtained by the ThisWork method
#define lOM1_ThisWork 3508.007474257327
#define lFM2_ThisWork 798.3572482927742
#define lQM3_ThisWork 339.0131651494688
#define aerfa1_ThisWork 0.029191397349496
#define beita1_ThisWork 0.066938381834233
#define gama1_ThisWork 0.922903028953325
#define rou1_ThisWork -1217.541934033181
#define rou2_ThisWork 1104.791627250177
#define rou3_ThisWork 554.7887202823558
#define fai1_ThisWork -0.616505221616897
#define fai2_ThisWork -0.708170549956763
#define fai3_ThisWork -1.505544580152118
#define Z1_ThisWork -69.199282491068420
#define Z2_ThisWork -13.657708803020116
#define Z3_ThisWork -198.7705155641413
#define k1_ThisWork -0.068080666045755
#define k2_ThisWork 0.020371843022479
#define p1_ThisWork -0.007618271064192
#define p2_ThisWork -0.004525671814249
#define k3_ThisWork 0.022321285382397
#define fx_ThisWork 813.2750642802467
#define u0_ThisWork 634.4840920542584
#define fy_ThisWork 900.5333297110174
#define v0_ThisWork 634.0638141247630


//ANN method related parameters
#define HDY_InMaxMinBucket  { 1204.566666666667, 524.7503333333333, 425.6746666666666, 322.4323333333334,1058.906666666667, 316.8083333333333, 470.5326666666667, 231.2256666666667,1056.14,58.8631,667.5066666666665,106.1186666666667}
#define HDY_OutMaxMinBucket { 0.670821189199015,-2.307036293787789 }
#define HDY_IWBucket {-0.389154083248616,-0.110577874373600,0.841275861386217,0.604302197373005,-0.496884160807103, -1.099161877969541,0.554480753715281,   0.118371831653325, - 1.172719703419154, - 0.667105285175155,   0.667874972729348,   1.289369719648501,0.009230574037060 ,  0.339590785710488 ,  0.905067127958263 ,- 1.707643116504798 ,- 0.938368182213069 ,  2.658641709199179,- 1.181985608015729 ,  0.270234364557230 ,  1.229847247169837, - 0.181136198476546 ,- 0.391526520839770 ,- 0.925322312020132,- 0.813500722497447 ,  0.118712292311899 ,  1.629878256464571, - 1.008906395558330 ,- 0.882205303204601,   1.781581760278714}
#define HDY_Lw1Bucket { -47.336100991281690, -30.769543664993041,  -2.453175275678196,  -1.850525667824383,  -4.372206473158050,- 36.594434175034763, - 21.363026806342546, - 2.036294135056538 ,  0.857717177473967 ,- 0.020722382016979,1.963150051132197,   2.485122262214316, - 0.452357324275466,   0.045735645979680,   0.550505715448815,- 1.988882238748835 ,  0.219225375791109 ,- 0.625619628264140,   0.258188022693620,   0.665591167583298,28.047188173870033 , 18.998044376841975,   2.706797186255157, - 3.003173229858799, - 3.168606666029340}
#define HDY_Lw2Bucket {0.670412713001073 ,  3.618387509722982, -15.516909698828815,   1.478216403810714 , -0.284498288498831,0.411660076553448 ,  1.517074436387944,  24.972286906741488 ,- 6.379590129434492 ,- 0.315372210158154,0.587508098610159 ,  2.693505960897007 , 18.651199909023209 ,- 6.160464770153880 ,- 0.444253404675163,0.150164945124439 ,- 0.175338885160413 , 37.893636999315888 ,- 7.640028721180705, - 0.212163099591996,1.023851684733126 ,  0.196402176935244 ,- 38.222468253862040 , 11.461106632827049 ,- 0.100168444773831}
#define HDY_Lw3Bucket { -1.788573296287180,10.331913216810811,  -5.008854527381015,  -2.608470595284970 , -2.512644217558703 }
#define HDY_b1Bucket { -1.266210822651797,1.359573324295312,0.132874621717964,0.582072039090469,- 0.242208293326912 }
#define HDY_b2Bucket { -14.082317794698270,- 12.911585548589837,- 1.565770239543380,- 1.892386395418907,5.921647173027711 }
#define HDY_b3Bucket { -8.153450971201968,20.314475889580649,16.039222486351694,29.371734089532385,- 30.777956127866393 }
#define HDY_b4Bucket -0.189951064540593
#define HDY_InMaxMinArm {  1204.566666666667, 524.7503333333333, 425.6746666666666, 322.4323333333334,1058.906666666667, 316.8083333333333, 470.5326666666667, 231.2256666666667}
#define HDY_OutMaxMinArm { -0.518932123496017,-2.632265606881574 }
#define HDY_IWArm {-1.001151737077650,   0.367257120985556 ,  1.175248031291057 ,  0.174851300276917,2.383574258630617, - 0.298685412382876 ,- 2.618557827160366 ,  0.198066051702386,- 2.518562345331556,   0.890297000716352,   2.842105169981565, - 2.491833895101840,- 1.415028140118889 ,  3.315114755875861 ,- 2.574898349800637 ,- 3.993997114518175,- 1.799207783171416,   0.266224242976773  , 2.079522491701995 ,  1.819533274378859}
#define HDY_Lw1Arm {0.143811079002289 , -6.825271919344857 , -2.484377962423221 , -3.552999673856588 , -1.256107733029046,- 3.383578374740606, - 4.182360981367842, - 0.651066285112758 ,  0.696510279290149,   1.533016723838240,- 6.448045472666820, - 6.438758589361715, - 2.197759463062825,   1.117698617389487,   7.819923043302440,- 3.000334605005549, - 2.575631937574209 ,- 1.175366849066971, - 0.184672891878254,   4.908008187462555,3.558366523773457,   3.137382339684176,   0.949162534624817 ,  0.248894204315395 ,- 3.671823307424087}
#define HDY_Lw2Arm {0.269903630032355,  -1.704861148959993 ,  1.133814115947475,   0.226265731319995,   0.955023498022967,- 0.022580600246203 ,  1.095216035499845, - 0.871353235248493 ,- 3.402890519092656, - 0.941954835266836,- 2.980067281396688,   3.723912353041764, - 0.462375415112087 ,  1.168917706801969 ,- 2.704218552335319,- 0.126780553959357 ,- 3.094052946843327 ,  2.199253680182953, - 0.029669266382860, - 1.370228639374954,0.863957914314236 ,  0.508332909921734, - 1.329351871803880  , 0.636414752579619  , 0.794769547872745}
#define HDY_Lw3Arm {3.407656505618620 , 9.941892277954164,  -0.127493019313718 ,  6.522337092731282,  -0.218533953984606}
#define HDY_b1Arm { 1.547531811022074,- 0.324100638084379,- 0.829492452911166,0.679936862669972,3.039923028617625}
#define HDY_b2Arm {3.455136602400832,0.171915951447378,- 3.908977228148623,- 3.974218285080870,1.344515737133793}
#define HDY_b3Arm {-0.546261026663893,- 1.993937147363598,0.692432962567515,- 0.884282168931355,- 0.282440001131688}
#define HDY_b4Arm 1.286377512531597
#define HDY_InMaxMinBoom { 1204.566666666667, 524.7503333333333, 425.6746666666666, 322.4323333333334}
#define HDY_OutMaxMinBoom  {0.534073381211097,-0.036976468061242  }
#define HDY_IWBoom {-3.227063507188007, 0.019838109156454,1.295639539389909 ,  3.133448977993644,1.254043378761577, - 2.335187980555810,- 2.229168803246588,   0.851117632709859,- 2.381772888410705, - 1.487021000572708}
#define HDY_Lw1Boom  {0.736136441864227  , 1.621860913713058 ,  0.700945185791202, - 0.298147645372492, - 0.241394060593899,- 0.791353059924057  , 0.341709733072567, - 0.679439136583555, - 1.498481145409551,   0.623556692517974,0.224227713500326, - 1.183428164355506,   0.921231282127512 ,- 1.009746573805637,   0.157389564814020,0.509881856544906 ,- 1.521004676064941, - 0.973414002055682 ,  0.520643068097176 ,  0.380640008196326,0.987819462233796, - 0.632834595983425 ,  1.040132056048107 ,- 0.067279149310760 ,  1.064950524081995}
#define HDY_Lw2Boom {-0.443304619355614, - 1.404065879639917, - 1.320751399763173, - 1.014472582138599 ,  0.160600875876728,0.557220684341762 ,- 1.318914279774366, - 0.061491311946452,   1.040048492624606, - 0.679381651820692,1.344432162563000,   0.270875711601082 ,- 0.264118529132838, - 0.905902875905349,   0.994262927593740,- 0.411368636521663,   1.095377704095019, - 1.426560689832397, - 0.295069640297422 ,  0.255573037856998,- 0.152526512876315,   1.108697196345159 ,- 0.429789543642482 ,- 0.736448097878391,   0.997007789760922}
#define HDY_Lw3Boom {-0.608217276889862 ,- 0.819563114742562 ,- 0.196872171135851,   0.968836431360376, - 0.606301534181624}
#define HDY_b1Boom {2.940409349602369,- 1.439362082647550,- 0.575110761681087,- 1.077014960579911,- 3.531996873385453}
#define HDY_b2Boom {-2.052119247457441,1.046137176143459,0.089884270253799,0.750707250523235,1.933628747210142}
#define HDY_b3Boom  {1.815413505029221,- 0.955167514068958,0.103721145564181,- 0.920065011670064,2.155278098118283}
#define HDY_b4Boom 0.428095251122405


//The inherent geometric parameters of excavators
#define ACX 47.234
#define BC 1735.79
#define CF 3720
#define BF 2355.84
#define BCF (double)(acos((double)(BC*BC+CF*CF-BF*BF)/2/BC/CF)*180/PI)
#define AC 509.791
#define BC 1735.79
#define DF 1765
#define DC 2539.36
#define DFC (double)(acos((double)(DF*DF+CF*CF-DC*DC)/2/DF/CF)*180/PI)
#define EF 515
#define FQ 1613
#define EQ 2089
#define EFQ (double)(acos((double)(EF*EF+FQ*FQ-EQ*EQ)/2/EF/FQ)*180/PI)
#define GN 1520
#define MN 417
#define NF 1360
#define GF 435
#define GNF (double)(acos((double)(GN*GN+NF*NF-GF*GF)/2/GN/NF)*180/PI)
#define NQ 255
#define FNQ (double)(acos((double)(NF*NF+NQ*NQ-FQ*FQ)/2/NF/NQ)*180/PI)
#define FQN (double)(acos((double)(FQ*FQ+NQ*NQ-NF*NF)/2/FQ/NQ)*180/PI)
#define KQ 310.04
#define MK 359
#define QV 1075
#define KV 1190
#define KQV (double)(acos((double)(KQ*KQ+QV*QV-KV*KV)/2/KQ/QV)*180/PI)
#define xC (double)0
#define yC (double)0
#define zC (double)0
#define xdb0 (double)1311
#define xdg0 (double)1300
#define xcd0 (double)1120
#define ACT (double)(47.234*PI/180)

// Displacement sensor parameters
#define Nboom 12.551
#define Narm 12.551
#define Nbucket 12.668
#define PositionBoom 683

struct XY
{
	double X;
	double Y;
};

struct JointAngle
{
	double Aerfa;
	double Beita;
	double Gama;
};

#endif