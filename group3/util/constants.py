import math
from klampt import vectorops,so3

# Constants that affect the state machine
INITIAL_STATE = "DONE"
REAL_VACUUM = True
REAL_PERCEPTION = True
SEGMENT = True
CALIBRATE_CAMERA = False
SELECT_REAL_BIN = False
HARDCODED_BIN = 'H'

# Constants related to printing output
PRINT_BLOBS = True
PRINT_LABELS_AND_SCORES = True
DEBUG_PERCEPTION = True

# Downsample ratio
STEP = 15

# Right arm configurations

#################################################################################

Q_SCAN_BIN_A = [0.6527088244995117, -1.1293933537902834, 0.3930825764465332, 0.9802137223388673, 0.06826214498291017, 1.5976409887573244, 0.021092235809326173]
Q_SCAN_BIN_A_HAYDEN = [1.1002477188537598, -0.8245146725463868, -0.13805827075195312, 0.7232719406616211, 1.7690633416076662, 1.275888518865967, -0.15186409782714844]
Q_BEFORE_SCAN_A = [[1.1106020891601562, -0.8038059319335938, -0.18829614149780274, 2.361563420251465, 0.00728640873413086, 0.02914563493652344, -0.0065194183410644535], [1.157388503137207, -0.7274903878234864, -0.15186409782714844, 0.7949855424133301, 0.013422331878662111, 1.517873987878418, 0.0122718462890625]]


Q_IK_SEED_A_KEVIN = [-0.04716990917358399, -1.366776880444336, 1.142048695275879, 1.4561312612365724, -0.004601942358398438, 0.09855826550903321, 0.0]

Q_IK_SEED_A = [0.2956747965270996, -0.7735098114074708, 1.5788497241271973, 1.2356215232299805, -2.3005876840026858, 0.7712088402282715, 1.3518205677795412]


#Transition from after scan to ik_seed is busted
Q_AFTER_SCAN_A = [[1.1884516140563965, -0.7359272821472168, -0.17487380961914065, 0.9572040105468751, 0.10316020786743164, 1.3430001782592775, -0.0487038899597168], [1.180014719732666, -0.7447476716674806, -0.13115535721435548, 2.2507333084533694, 0.10469418865356446, 0.042951462011718754, -0.04947088035278321],
[0.7271068926269532, -0.7255729118408204, -0.06864564017944337, 2.302888655181885, -0.13882526114501953, -0.08206797205810547, 0.07439806812744142],
[0.6661311563781739, -0.6845389258117677, -0.013805827075195313, 2.1863061154357912, -1.8983012228393557, 0.06634466900024415, 1.5677283634277346],
[0.46364569260864263, 0.11696603494262696, 1.5504710795837404, 1.9504565695678713, -1.7510390673706056, 1.5661943826416016, 1.597257493560791],
[0.43181559129638675, -0.8333350620666504, 1.680092456011963, 1.8300390778564455, -0.7405292245056153, 1.5401167092773438, 1.598791474346924],
[0.1334563283935547, -0.8693836105407715, 2.0543837678283694, 1.7333982883300783, -2.282563409765625, 1.4381069869995118, 1.3506700821899416]]
Q_AFTER_GRASP_A = [
[0.1334563283935547, -0.8693836105407715, 2.0543837678283694, 1.7333982883300783, -2.282563409765625, 1.4381069869995118, 1.3506700821899416],
[0.2876213973999024, -0.781563210534668, 1.5546895267456056, 2.37536924732666, -1.7786507215209961, 1.0799224734375001, 1.810097327636719],
[0.7497331092224122, -1.062665189593506, 0.17525730481567384, 2.4225391565002443, -1.2011069555419922, -0.2899223685791016, 0.16490293450927734],
[1.2129953066345216, -0.042951462011718754, 0.02300971179199219, 1.7054031389831543, 0.12732040524902344, -0.3516650952209473, 0.18983012228393556]
]

################################################################################################
Q_SCAN_BIN_B = [0.5790777467651368, -1.1336118009521485, 0.13729128035888674, 0.8176117590087891, -0.05407282271118164, 1.908655593145752, 0.024543692578125]
Q_SCAN_BIN_B_HAYDEN = [0.8298836052978517, -0.8214467109741211, -0.194432064642334, 0.3762087877990723, 0.16298545852661134, 1.9995439547241212, -0.1464951650756836]
Q_IK_SEED_B = [-0.0851359336303711, -1.3744467843750001, 0.8548107930725098, 1.4568982516296387, -0.01457281746826172, -0.04525243319091797, 0.0]
#Elbow is down - makes for best configuration

Q_AFTER_SCAN_B = [
[-1.3790487267333986, -1.3449176542419434, 2.281796419372559, 1.0580632472351075, -0.08321845764770508, 1.5846021520751954, 0.09280583756103516],
[-1.41394678961792, -1.2413739511779787, 2.2756604962280274, 1.7391507162780764, -0.11965050131835939, -0.08168447686157228, 0.06212622183837891]
]


#Q_AFTER_SCAN_B_OLD = [
#[0.977912751159668, -0.5744758044067383, -0.12616991965942384, 2.119577951239014, -0.03451456768798828, 0.021475731005859377, 0.04985437554931641],
#[0.05253884192504883, -0.7106165991760255, -0.024160197381591798, 2.048247844683838, 3.0595246779418948, -0.3501311144348145, 0.11198059738769532],
#[0.1714223528503418, -0.783480686517334, -0.06596117380371094, 2.2886993329101566, 3.060291668334961, 1.447694366912842, 3.049170307635498],
#[-0.01687378864746094, -0.5376602655395508, -0.1449611842895508, 1.9980099739379884, 0.7926845712341309, 1.7023351774108888, 0.005368932751464844],
#[0.15263108822021484, 0.43565054326171876, 3.0062188456237795, 2.089665325909424, 2.5287673259399415, 1.6973497398559572, 0.32021848910522466],
#[0.7386117485229493, 0.3305728594116211, 2.9759227250976563, 2.1886070866149905, 3.0587576875488285, 1.6927477974975587, 0.3393932489318848]]

Q_BEFORE_SCAN_B = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]

Q_AFTER_GRASP_B = [
[-1.41394678961792, -1.2413739511779787, 2.2756604962280274, 1.7391507162780764, -0.11965050131835939, -0.08168447686157228, 0.06212622183837891],
[-1.3134710481262208, -1.2294856000854493, 2.2541847652221683, 1.0208642131713868, -0.012655341485595705, 1.2429079319641114, 0.07631554411010742],
[-0.8091748646850586, -1.4170147511901856, 1.6566992490234376, 2.4018304158874515, -0.12693691005249025, 0.5537670637939454, 0.1721893432434082], 
[0.6761020314880372, -1.1148205363220216, 0.0364320436706543, 2.4739275128356937, -0.053689327514648444, -0.030296120526123047, 0.02185922620239258]
]


################################################################################################
Q_SCAN_BIN_C = [0.6684321275573731, -1.10983509876709, -0.2672961519836426, 0.7593204891357422, -0.0019174759826660157, 1.8457623809143067, -0.28915537818603515]
Q_SCAN_BIN_C_HAYDEN = [0.9088836157836915, -0.7075486376037599, -0.9472331354370118, 0.7884661240722657, -0.8413884611938477, 1.1017816996398926, -0.1342233187866211]

Q_IK_SEED_C = [0.36393694151000977, -1.2198982201721191, -0.24121847861938478, 1.1443496664550783, 1.7955245101684572, 0.5399612367187501, -1.64979633548584]


Q_AFTER_SCAN_C = [
[0.5829126987304688, -1.1627574358886719, -0.07056311616210938, 2.4474663442749027, -0.010737865502929688, 0.2439029449951172, 0.11658253974609376],
[-0.45789326466064456, -1.4058933904907227, 0.12847089083862306, 1.5378157380981445, -0.018024274237060548, 1.56120894508667, -1.5358982621154786],
[-0.45981074064331057, -1.4135632944213867, 0.2684466375732422, 1.4338885398376466, 1.6816264367980958, 1.1784807389465333, -1.6946652734802248]
]

Q_BEFORE_SCAN_C = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]
Q_AFTER_GRASP_C = [
[-0.45981074064331057, -1.4135632944213867, 0.2684466375732422, 1.4338885398376466, 1.6816264367980958, 1.1784807389465333, -1.6946652734802248],
[0.8605632210205079, -1.000538967755127, 0.031446606115722656, 2.139136206262207, 0.04448544279785156, 0.5042961834411621, -0.029912625329589847]
 ]

################################################################################################
Q_SCAN_BIN_D = [1.0561457712524416, -0.8613302114135742, -0.15454856420288088, 0.9568205153503418, 0.09012137118530274, 1.597257493560791, 0.21552430045166018]
Q_SCAN_BIN_D_HAYDEN = [1.4028254289184572, -0.4571262742675782, -1.0285341171020508, 0.5234709432678223, 0.8716845817199708, 1.7142235285034182, -0.03067961572265625]
Q_IK_SEED_D = [0.4421699616027832, -1.2264176385131837, 0.5875146410888672, 1.7740487791625978, 0.12540292926635743, -0.4720825869323731, 0.0]

Q_AFTER_SCAN_D = [
[0.6408204734069824, -0.8571117642517091, -0.042184471618652346, 2.408349834228516, -0.05560680349731446, -0.09664078952636719, 0.18024274237060547],
[0.2634612000183106, -1.2916118219238282, 0.93266031796875, 2.443247897113037, 0.6860729065979004, 0.19059711267700197, -0.3535825712036133],
[0.2822524646484375, -1.2870098795654297, 1.2133788018310547, 2.3531265259277347, 0.615509790435791, -1.379815717126465, -0.4755340437011719],
]

Q_BEFORE_SCAN_D = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]

Q_AFTER_GRASP_D = [
[0.2822524646484375, -1.2870098795654297, 1.2133788018310547, 2.3531265259277347, 0.615509790435791, -1.379815717126465, -0.4755340437011719],
[-0.13920875634155275, -1.310403086553955, 1.1708108350158692, 2.4589712001708985, -1.038121497015381, -0.9675583808532715, 0.5042961834411621],
[0.6408204734069824, -0.8571117642517091, -0.042184471618652346, 2.408349834228516, -0.05560680349731446, -0.09664078952636719, 0.18024274237060547]
]

################################################################################################
Q_SCAN_BIN_E = [0.8582622498413086, -1.0009224629516602, -0.2201262428100586, 1.2268011337097169, 0.07209709694824219, 1.3552720245483398, -0.2676796471801758]
Q_SCAN_BIN_E_HAYDEN = [0.7574030131530762, -0.37007286465454103, 0.09127185677490235, 0.026077673364257814, -0.09549030393676758, 1.9182429730590822, -0.01112136069946289]
Q_IK_SEED_E = [-0.33594179216308595, -0.06864564017944337, 1.7583254761047364, 1.92399540100708, 0.10200972227783203, -0.8463738987487793, -1.6030099215087892]
Q_AFTER_SCAN_E = [
[-0.03988350043945313, -1.36485940446167, 0.8563447738586426, 1.9792187093078615, -0.03298058690185547, 1.023165184350586, 0.20095148298339846],
[-0.4329660768859864, -1.3318788175598146, 1.397073000970459, 2.6177382115356447, 2.579005196685791, -0.5533835685974121, -2.119577951239014],
[-0.5250049240539552, -0.08628641921997071, 1.762927418463135, 2.5866751006164552, 1.6195002149597169, -1.568878849017334, -1.6114468158325197],
[-0.5729418236206055, -0.05062136594238282, 1.876441996636963, 2.4628061521362308, 0.2051699301452637, -1.1355292769348144, -1.613747787011719]
]
Q_BEFORE_SCAN_E = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]

Q_AFTER_GRASP_E = [
[-0.5729418236206055, -0.05062136594238282, 1.876441996636963, 2.4628061521362308, 0.2051699301452637, -1.1355292769348144, -1.613747787011719],
[-0.5595194917419434, -0.04180097642211914, 1.508286607965088, 2.5095925661132816, 0.7144515511413575, -1.211844821044922, -1.6118303110290528],
[0.18906313189086915, -0.8935438079223633, 0.8034224367370606, 2.358111963482666, 1.2831749276000977, -0.4368010288513184, -1.565043897052002],
[0.5579855109558106, -0.6596117380371094, 0.25847576246337894, 2.121878922418213, -0.06442719301757813, -0.10316020786743164, 0.014189322271728517]
]

################################################################################################
# shortcutting may be possible
Q_SCAN_BIN_F = [0.6419709589965821, -1.078004997454834, -0.2933738253479004, 1.3782817363403321, 0.015339807861328126, 1.2256506481201173, -0.15301458341674806]
Q_SCAN_BIN_F_HAYDEN =  [0.5867476506958008, -0.3140825659606934, -0.17525730481567384, 0.3758252926025391, -1.4787574778320314, 1.2064758882934572, -0.04486893799438477]
Q_IK_SEED_F = [0.13652428996582033, -0.9610389625122071, 0.09395632315063478, 1.5823011808959961, -0.6477233869445801, -0.9173205101074219, 0.49777676510009766]
Q_AFTER_SCAN_F = [[-0.08897088559570313, -1.3464516350280762, 0.0724805921447754, 2.351976040338135, -1.0208642131713868, 0.1695048768676758, 0.28301945504150394],
[-0.14879613625488283, -1.3832671738952638, 0.16221846813354493, 2.309024578326416, -0.7370777677368164, -1.2406069607849122, 0.49010686116943364]
]
Q_BEFORE_SCAN_F = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]
Q_AFTER_GRASP_F = [[-0.14879613625488283, -1.3832671738952638, 0.16221846813354493, 2.309024578326416, -0.7370777677368164, -1.2406069607849122, 0.49010686116943364],
[-0.061742726641845706, -1.3541215389587404, 0.4705486061462403, 2.5847576246337893, -0.21207284368286133, -0.1817767231567383, -0.120800986907959],
[0.9748447895874024, -0.9073496349975586, 0.23700003145751955, 2.4716265416564944, 0.2323980890991211, -0.28493693102416995, -0.03451456768798828]
]

###############################################################################################
Q_SCAN_BIN_G = [1.1976554987731935, -0.7267233974304199, -0.2937573205444336, 1.2540292926635743, 0.26806314237670903, 1.2183642393859864, 0.33018936421508793]
Q_SCAN_BIN_G_HAYDEN = [1.1508690847961427, -0.2193592524169922, -0.13499030917968752, 0.43526704806518557, 0.1683543912780762, 1.329194351184082, 0.24927187774658205]
Q_IK_SEED_G = [0.9123350725524902, -0.9660244000671387, 0.33018936421508793, 1.959276959088135, 0.3420777153076172, -1.0028399389343263, -0.4061214131286621]
Q_AFTER_SCAN_G = [
	[1.1036991756225587, -0.8214467109741211, 0.22281070918579102, 2.2327090342163087, 0.33210684019775394, -0.08935438079223633, -0.1852281799255371],
	[0.9545195441711426, -0.9376457555236817, 0.41685927863159183, 2.3189954534362793, 0.4839709380249024, -1.570796325, -0.18752915110473634],
	[0.9330438131652833, -0.9541360489746095, 0.37314082622680667, 2.0961847442504884, 0.40957286989746094, -1.1961215179870606, -0.19059711267700197]
]

Q_BEFORE_SCAN_G = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]
]
Q_AFTER_GRASP_G = [
	[0.9330438131652833, -0.9541360489746095, 0.37314082622680667, 2.0961847442504884, 0.40957286989746094, -1.1961215179870606, -0.19059711267700197],
	[0.7907670952514649, -1.042339944177246, 0.41839325941772465, 2.2863983617309573, 0.2914563493652344, -1.1922865660217286, -0.20095148298339846],
	[0.7232719406616211, -1.0868253869750977, 0.26116022883911133, 2.5847576246337893, -0.3390097537353516, -1.457281746826172, 0.005368932751464844],
	[0.9019807022460938, -0.7708253450317384, 0.21360682446899415, 2.207014856048584, -0.3443786864868164, -0.3555000471862793, 0.050237870745849615]
]

################################################################################################
Q_BEFORE_SCAN_H = []
Q_SCAN_BIN_H = [0.8390874900146484, -0.9054321590148926, -0.08743690480957032, 1.6279371092834474, -0.015339807861328126, 0.9142525485351564, -0.16298545852661134]
Q_IK_SEED_H = [0.8567282690551759, -0.8348690428527833, -0.17602429520874024, 1.8956167564636233, 0.04256796681518555, -1.1719613206054689, 0.0]
Q_AFTER_SCAN_H = [
	[0.5740923092102052, -1.1692768542297365, 0.12923788123168947, 2.5866751006164552, -0.06941263057250976, 0.02185922620239258, 0.0],
	[-0.26192721923217777, -1.2793399756347656, 0.053689327514648444, 2.5839906342407226, -0.32328645067749023, 0.05483981310424805, 0.00613592314453125],
	[-0.03719903406372071, -1.2919953171203613, -0.03451456768798828, 2.5475585905700684, -1.143966171258545, -1.3188399808776856, 0.06902913537597656],
	[0.694509800921631, -0.9898011022521973, -0.05944175546264649, 2.323980890991211, -0.24505343058471682, -1.3901700874328615, 0.0]
]

Q_AFTER_GRASP_H = [
	[0.694509800921631, -0.9898011022521973, -0.05944175546264649, 2.323980890991211, -0.24505343058471682, -1.3901700874328615, 0.0],
	[-0.03719903406372071, -1.2919953171203613, -0.03451456768798828, 2.5475585905700684, -1.143966171258545, -1.3188399808776856, 0.06902913537597656],
	[0.3543495615966797, -1.236388513623047, -0.07056311616210938, 2.5000051861999513, -1.447694366912842, -0.6005534777709961, 0.9572040105468751],
	[1.12900985859375, -0.5319078375915528, -0.10431069345703126, 1.9573594831054688, -1.2586312350219728, -0.13499030917968752, 1.2260341433166504]
]

################################################################################################
Q_SCAN_BIN_I = [0.5691068716552735, -0.7850146673034668, -0.20401944455566406, 1.3832671738952638, 0.07094661135864258, 1.0415729537841798, -0.13192234760742189]
Q_IK_SEED_I = [0.24965537294311524, -0.9196214812866211, 0.1928980838562012, 1.9285973433654786, -0.2316310987060547, -1.0223981939575195, 0.00728640873413086]
Q_AFTER_SCAN_I = [
	[0.1353738043762207, -1.0177962515991212, -0.13729128035888674, 2.2515002988464357, -0.7466651476501466, 0.11619904454956055, 0.09510680874023437],
	[0.1338398235900879, -1.3280438655944826, -0.16566992490234375, 2.5874420910095215, -0.8874078847778321, -1.3430001782592775, 0.09549030393676758],
	[0.39423306203613284, -1.244058417553711, -0.17487380961914065, 2.1544760141235355, -0.7953690376098633, -0.8571117642517091, 0.3060291668334961]
]

Q_BEFORE_SCAN_I = []

Q_AFTER_GRASP_I = [
	[0.39423306203613284, -1.244058417553711, -0.17487380961914065, 2.1544760141235355, -0.7953690376098633, -0.8571117642517091, 0.3060291668334961],
	[0.1338398235900879, -1.3280438655944826, -0.16566992490234375, 2.5874420910095215, -0.8874078847778321, -1.3430001782592775, 0.09549030393676758],
	[0.5215534672851563, -1.1922865660217286, -0.15301458341674806, 2.5928110237609867, -1.3656263948547365, -1.1029321852294922, 0.169888372064209],
	[0.6561602812683106, -0.702179704852295, 0.21322332927246096, 2.2779614674072266, -0.3654709222961426, -0.34667965766601566, -0.06404369782104492]
]

################################################################################################
Q_SCAN_BIN_J = [1.3989904769531252, -0.3125485851745606, -0.49585928911743166, 1.1167380123046875, 0.6891408681701661, 1.1336118009521485, 0.07554855371704101]
Q_SCAN_BIN_J_HAYDEN = [1.0143447948303224, 0.25118935372924805, 0.37083985504760747, 0.21207284368286133, -0.32328645067749023, 1.208393364276123, 0.14419419389648439]
Q_IK_SEED_J = [1.196888508380127, -0.3286553834289551, -0.014956312664794923, 1.5826846760925295, 0.30948062360229495, -1.2578642446289063, -0.04486893799438477]
Q_AFTER_SCAN_J = [
	[1.508286607965088, -0.4007524803771973, -0.08321845764770508, 2.1794032018981935, 0.5928835738403321, -0.4287476297241211, 0.10507768385009766],
	[1.5182574830749513, -0.4252961729553223, -0.03873301484985352, 2.1479565957824707, 0.6139758096496583, -1.5692623442138673, 0.09127185677490235],
	[1.134378791345215, -0.7781117537658692, 0.04985437554931641, 2.273359525048828, 0.12885438603515625, -1.278189490045166, 0.021092235809326173]
]
Q_BEFORE_SCAN_J = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]

Q_AFTER_GRASP_J = [
	[1.134378791345215, -0.7781117537658692, 0.04985437554931641, 2.273359525048828, 0.12885438603515625, -1.278189490045166, 0.021092235809326173],
	[1.5182574830749513, -0.4252961729553223, -0.03873301484985352, 2.1479565957824707, 0.6139758096496583, -1.5692623442138673, 0.09127185677490235],
	[1.510204083947754, -0.41302432666625977, -0.08130098166503907, 2.1326167879211426, 0.47706802448730473, -0.6826214498291016, 0.11773302533569337]
]

################################################################################################
Q_SCAN_BIN_K = [1.034670040246582, -0.3857961677124024, -0.36930587426147465, 1.330728331970215, 0.3742913118164063, 0.8774370096679688, -0.14841264105834961]
Q_SCAN_BIN_K_HAYDEN = [0.8057234079162598, 0.17985924717407228, -0.0609757362487793, 0.37390781661987305, -0.0015339807861328126, 1.0166457660095216, -0.016106798254394532]
Q_IK_SEED_K = [0.598252506591797, -0.4003689851806641, 0.10661166463623048, 1.8185342219604494, -0.17985924717407228, -1.357189500531006, -0.002300971179199219]
Q_AFTER_SCAN_K = [[0.822980691760254, -1.0285341171020508, -0.07401457293090821, 2.4052818726562504, -0.034898062884521484, 0.2653786760009766, -0.02531068297119141],
[0.7819467057312012, -0.6914418393493653, -0.04601942358398438, 2.1847721346496582, -0.04026699563598633, -1.2570972542358398, -0.027228158953857422]]
Q_BEFORE_SCAN_K = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]

Q_AFTER_GRASP_K = [[0.7819467057312012, -0.6914418393493653, -0.04601942358398438, 2.1847721346496582, -0.04026699563598633, -1.2570972542358398, -0.027228158953857422],
[0.9522185729919435, -0.9318933275756837, 0.008820389520263672, 2.357728468286133, -0.0364320436706543, -0.22587867075805665, 0.09664078952636719]
]

################################################################################################
Q_SCAN_BIN_L = [0.6615292140197754, -0.3497476192382813, -0.32251946028442385, 1.1604564647094728, 0.19328157905273438, 0.9909515878417969, -0.17525730481567384]
Q_SCAN_BIN_L_HAYDEN = [-0.08858739039916992, 0.42069423059692385, 1.285475898779297, 0.8187622445983888, -1.8158497555847168, 1.0561457712524416, -0.09012137118530274]
Q_IK_SEED_L = [0.09855826550903321, -0.3010437292785645, 0.18983012228393556, 1.6264031284973146, -0.6109078480773926, -1.302733182623291, -0.0015339807861328126]

Q_AFTER_SCAN_L = [
	[0.0847524384338379, -0.25847576246337894, 0.0011504855895996095, 1.7905390726135255, -0.7478156332397461, -0.06250971703491211, -0.10507768385009766],
	[0.08015049607543946, -0.30794664281616213, 0.04755340437011719, 1.802427423706055, -0.6331505694763184, -1.4956312664794922, -0.06250971703491211]
]

Q_BEFORE_SCAN_L = [
	[0.6876068873840332, -0.22396119477539064, -0.033364082098388675, 1.864937140740967, -0.3175340227294922, -0.26116022883911133, 0.1100631214050293]
]

Q_AFTER_GRASP_L = [
	[0.08015049607543946, -0.30794664281616213, 0.04755340437011719, 1.802427423706055, -0.6331505694763184, -1.4956312664794922, -0.06250971703491211],
	[0.15953400175781252, -0.4506068559265137, 0.04448544279785156, 2.4225391565002443, -0.830650595690918, -1.2935292979064943, -0.0732475825378418]

]

################################################################################################
Q_STOW = [1.0, 0.28, 0.0, 0.9, 0.0, 0.0, 0.0]

# Right arm calibration configs, see pics on Google Drive
Q_CALIBRATE_BIN_E = [1.198805984362793, -0.551082597418213, -0.4406359808166504, 1.0392719826049805, 0.42069423059692385, -0.5196359913024903, -0.14189322271728516]
Q_CALIBRATE_BIN_H = [0.9088836157836915, -0.1096796262084961, -0.46364569260864263, 0.6841554306152344, 0.5288398760192872, -0.3531990760070801, -0.15531555459594729]

# 7 list, each element is a 2 list of lower then upper acceptable bound
ELBOW_UP_BOUNDS = [[-10, 10], [-10, 10], [-1.5, 1.5], [-10, 10], [-.5, .5], [-10, 10], [-10, 10]]

# Number of histograms per object
NUM_HIST_PER_OBJECT = 4

# Left arm configurations
Q_SPATULA_AT_BIN = [0.08053399127197267, 0.5610534725280762, -1.4894953433349611, 0.830650595690918, -2.828277074432373, -1.3989904769531252, 0.0]

# Paths
REPO_ROOT = "/home/group3/ece490-s2016"
KLAMPT_MODELS_DIR = REPO_ROOT + "/apc2015/klampt_models/"
LIBPATH = REPO_ROOT + "/common/"
VACUUM_PCD_FILE = REPO_ROOT + "/group3/planning/custom_vacuum.pcd"
PICK_JSON_PATH = REPO_ROOT + '/group3/integration/apc_pick_task.json'
PERCEPTION_DIR = REPO_ROOT + "/group3/perception"
BIN_NPZ_FOLDER = PERCEPTION_DIR + "/empty_bins/" # A.npz, B.npz
MAT_PATH = PERCEPTION_DIR + "/matpcl/"
CLOUD_MAT_PATH = MAT_PATH + "cloud.mat"
CHENYU_GO_PATH = MAT_PATH + "chenyugo.txt"
CHENYU_DONE_PATH = MAT_PATH + "chenyudone.txt"
ARDUINO_SERIAL_PORT = "/dev/ttyACM0"
ROS_DEPTH_TOPIC = "/realsense/pc"
PICK_JSON_PATH = REPO_ROOT + '/group3/planning/apc_pick_task.json'

# Indices in Baxter robot file link
LEFT_CAMERA_LINK_NAME = 'left_hand_camera'
RIGHT_CAMERA_LINK_NAME = 'right_hand_camera'
LEFT_GRIPPER_LINK_NAME = 'left_gripper'
RIGHT_GRIPPER_LINK_NAME = 'right_gripper'
LEFT_ARM_LINK_NAMES = ['left_upper_shoulder','left_lower_shoulder','left_upper_elbow','left_lower_elbow','left_upper_forearm','left_lower_forearm','left_wrist']
RIGHT_ARM_LINK_NAMES = ['right_upper_shoulder','right_lower_shoulder','right_upper_elbow','right_lower_elbow','right_upper_forearm','right_lower_forearm','right_wrist']

# Local transform from right_wrist to vacuum point
VACUUM_POINT_XFORM = (so3.identity(), [.04, -0.03, .43])

# Local transform from right_lower_forearm to F200
RIGHT_F200_CAMERA_CALIBRATED_XFORM = ([0.03704561639240232, -0.03280371662154699, -0.9987750189516768, 0.990578526979957, 0.13306496892712172, 0.032371220715998364, 0.1318400705109292, -0.9905642988823286, 0.03742413108458098], [0.13, -0.14, -0.06499999999999999])

# Transform of shelf relative to world
SHELF_MODEL_XFORM = [1.43,-.32,-.03]

# Distances (meters)
GRASP_MOVE_DISTANCE = .07
COM_ADJUSTMENT = [.01, 0.0, GRASP_MOVE_DISTANCE]
COM_Y_ADJUSTMENT = 0.0
MEAN_OBJ_HEIGHT = .05
BACK_UP_DISTANCE = .2

# Times (seconds)
MOVE_TIME = 2
SCAN_WAIT_TIME = 3 if REAL_PERCEPTION else 0
GRASP_WAIT_TIME = 2 if REAL_VACUUM else 0

# Terminal colors
OKBLUE = '\033[94m'
OKGREEN = '\033[92m'
FAIL_COLOR = '\033[91m'
WARNING_COLOR = '\033[93m'
END_COLOR = '\033[0m'


ITEM_SCORES = {
    "i_am_a_bunny_book":1,
    "laugh_out_loud_joke_book":1,
    "scotch_bubble_mailer":1,
    "up_glucose_bottle":1,
    "dasani_water_bottle":1,
    "rawlings_baseball":1,
    "folgers_classic_roast_coffee":1,
    "elmers_washable_no_run_school_glue":1,
    "hanes_tube_socks":1,
    "womens_knit_gloves":1,
    "cherokee_easy_tee_shirt":1,
    "peva_shower_curtain_liner":1,
    "cloud_b_plush_bear":1,
    "barkely_hide_bones":1,
    "kyjen_squeakin_eggs_plush_puppies":1,
    "cool_shot_glue_sticks":1,
    "creativity_chenille_stems":1,
    "soft_white_lightbulb":1,
    "safety_first_outlet_plugs":1,
    "oral_b_toothbrush_green":1,
    "dr_browns_bottle_brush":1,
    "command_hooks":1,
    "easter_turtle_sippy_cup":1,
    "fiskars_scissors_red":1,
    "scotch_duct_tape":1,
    "woods_extension_cord":1,
    "platinum_pets_dog_bowl":1,
    "fitness_gear_3lb_dumbbell":1,
    "rolodex_jumbo_pencil_cup":1,
    "clorox_utility_brush":1,
    "kleenex_paper_towels":1,
    "expo_dry_erase_board_eraser":1,
    "kleenex_tissue_box":1,
    "ticonderoga_12_pencils":1,
    "crayola_24_ct":1,
    "jane_eyre_dvd":1,
    "dove_beauty_bar":1,
    "staples_index_cards":1,
    "oral_b_toothbrush_red": 1
}

ITEM_NUMBERS = {
    "i_am_a_bunny_book":1,
    "laugh_out_loud_joke_book":2,
    "scotch_bubble_mailer":3,
    "up_glucose_bottle":4,
    "dasani_water_bottle":5,
    "rawlings_baseball":6,
    "folgers_classic_roast_coffee":7,
    "elmers_washable_no_run_school_glue":8,
    "hanes_tube_socks":9,
    "womens_knit_gloves":10,
    "cherokee_easy_tee_shirt":11,
    "peva_shower_curtain_liner":12,
    "cloud_b_plush_bear":13,
    "barkely_hide_bones":14,
    "kyjen_squeakin_eggs_plush_puppies":15,
    "cool_shot_glue_sticks":16,
    "creativity_chenille_stems":17,
    "soft_white_lightbulb":18,
    "safety_first_outlet_plugs":19,
    "oral_b_toothbrush_green":20,
    "dr_browns_bottle_brush":21,
    "command_hooks":22,
    "easter_turtle_sippy_cup":23,
    "fiskars_scissors_red":24,
    "scotch_duct_tape":25,
    "woods_extension_cord":26,
    "platinum_pets_dog_bowl":27,
    "fitness_gear_3lb_dumbbell":28,
    "rolodex_jumbo_pencil_cup":29,
    "clorox_utility_brush":30,
    "kleenex_paper_towels":31,
    "expo_dry_erase_board_eraser":32,
    "kleenex_tissue_box":33,
    "ticonderoga_12_pencils":34,
    "crayola_24_ct":35,
    "jane_eyre_dvd":36,
    "dove_beauty_bar":37,
    "staples_index_cards":38,
    "oral_b_toothbrush_red":39
}
