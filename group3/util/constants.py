import math
from klampt import vectorops,so3

# Constants that affect the state machine
INITIAL_STATE = "FAKE_PATH_PLANNING"
REAL_VACUUM = False
REAL_PERCEPTION = True
SEGMENT = False
CALIBRATE = False
IGNORE_ERROR = False
SELECT_REAL_BIN = False
HARDCODED_BIN = 'D'

# Constants related to printing output
PRINT_BLOBS = True
PRINT_LABELS_AND_SCORES = True
DEBUG_PERCEPTION = True

# Downsample ratio
STEP = 15

# Right arm configurations
Q_SCAN_BIN_A = [0.6527088244995117, -1.1293933537902834, 0.3930825764465332, 0.9802137223388673, 0.06826214498291017, 1.5976409887573244, 0.021092235809326173]
Q_SCAN_BIN_A_HAYDEN = [1.1002477188537598, -0.8245146725463868, -0.13805827075195312, 0.7232719406616211, 1.7690633416076662, 1.275888518865967, -0.15186409782714844]
Q_BEFORE_SCAN_A = [[1.1106020891601562, -0.8038059319335938, -0.18829614149780274, 2.361563420251465, 0.00728640873413086, 0.02914563493652344, -0.0065194183410644535], [1.157388503137207, -0.7274903878234864, -0.15186409782714844, 0.7949855424133301, 0.013422331878662111, 1.517873987878418, 0.0122718462890625]]
Q_IK_SEED_A = [-0.04716990917358399, -1.366776880444336, 1.142048695275879, 1.4561312612365724, -0.004601942358398438, 0.09855826550903321, 0.0]
Q_AFTER_SCAN_A = [[1.1884516140563965, -0.7359272821472168, -0.17487380961914065, 0.9572040105468751, 0.10316020786743164, 1.3430001782592775, -0.0487038899597168], [1.180014719732666, -0.7447476716674806, -0.13115535721435548, 2.2507333084533694, 0.10469418865356446, 0.042951462011718754, -0.04947088035278321],
[0.7271068926269532, -0.7255729118408204, -0.06864564017944337, 2.302888655181885, -0.13882526114501953, -0.08206797205810547, 0.07439806812744142],
[0.6661311563781739, -0.6845389258117677, -0.013805827075195313, 2.1863061154357912, -1.8983012228393557, 0.06634466900024415, 1.5677283634277346],
[0.46364569260864263, 0.11696603494262696, 1.5504710795837404, 1.9504565695678713, -1.7510390673706056, 1.5661943826416016, 1.597257493560791],
[0.43181559129638675, -0.8333350620666504, 1.680092456011963, 1.8300390778564455, -0.7405292245056153, 1.5401167092773438, 1.598791474346924],
[0.44715539915771485, -0.9015972070495606, 1.868772092706299, 1.515573016699219, -2.2829469049621585, 1.5604419546936037, 1.4641846603637696]]
Q_AFTER_GRASP_A = [
[0.44715539915771485, -0.9015972070495606, 1.868772092706299, 1.515573016699219, -2.2829469049621585, 1.5604419546936037, 1.4641846603637696],
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
[0.977912751159668, -0.5744758044067383, -0.12616991965942384, 2.119577951239014, -0.03451456768798828, 0.021475731005859377, 0.04985437554931641],
[0.05253884192504883, -0.7106165991760255, -0.024160197381591798, 2.048247844683838, 3.0595246779418948, -0.3501311144348145, 0.11198059738769532],
[0.1714223528503418, -0.783480686517334, -0.06596117380371094, 2.2886993329101566, 3.060291668334961, 1.447694366912842, 3.049170307635498],
[-0.01687378864746094, -0.5376602655395508, -0.1449611842895508, 1.9980099739379884, 0.7926845712341309, 1.7023351774108888, 0.005368932751464844],
[0.15263108822021484, 0.43565054326171876, 3.0062188456237795, 2.089665325909424, 2.5287673259399415, 1.6973497398559572, 0.32021848910522466],
[0.7386117485229493, 0.3305728594116211, 2.9759227250976563, 2.1886070866149905, 3.0587576875488285, 1.6927477974975587, 0.3393932489318848]]
Q_BEFORE_SCAN_B = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]
Q_AFTER_GRASP_B = [
[0.7386117485229493, 0.3305728594116211, 2.9759227250976563, 2.1886070866149905, 3.0587576875488285, 1.6927477974975587, 0.3393932489318848],
[0.6565437764648437, 0.6845389258117677, 2.839398435131836, 2.615820735552979, 2.9927965137451173, 2.0363594935913087, 0.2895388733825684],
[0.7857816576965333, -0.39538354762573247, 0.2059369205383301, 2.4309760508239746, 3.0591411827453614, 1.3046506586059572, 3.052238269207764],
[1.2160632682067871, -0.509665116192627, -0.08245146725463867, 2.106539114556885, 0.1580000209716797, -0.4839709380249024, 0.22626216595458987],
]

################################################################################################
Q_SCAN_BIN_C = [0.554917549383545, -1.132461315362549, -0.2692136279663086, 0.8379370044250489, 0.03834951965332031, 1.8112478132263186, -0.5276893904296875]
Q_SCAN_BIN_C_HAYDEN = [0.9088836157836915, -0.7075486376037599, -0.9472331354370118, 0.7884661240722657, -0.8413884611938477, 1.1017816996398926, -0.1342233187866211]

Q_IK_SEED_C = [-0.0970242847229004, -1.374063289178467, 0.6158932856323243, 1.1899855948425293, 0.039500005242919925, 0.31178159478149414, 0.0]

Q_AFTER_SCAN_C = [[0.20133497817993165, -1.3107865817504885, 0.0847524384338379, 0.9422476978820802, 0.14534467948608398, 1.9872721084350586, -0.5687233764587403],
[1.008208871685791, -0.6661311563781739, -0.08091748646850587, 2.256102241204834, 0.054456317907714845, -0.0602087458557129, 0.1342233187866211], 
[0.11619904454956055, -0.7715923354248048, 0.003067961572265625, 2.082378917175293, -2.280645933782959, -0.10047574149169923, 1.9362672472961426], 
[0.5495486166320801, -0.9341942987548829, -1.2218156961547852, 1.41394678961792, -2.5118935372924804, -1.459199222808838, 1.549704089190674], 
[0.5694903668518067, -1.170427339819336, -0.9575875057434082, 1.3602574621032715, -1.1616069502990722, -1.4733885450805666, 1.5428011756530762]
]
Q_BEFORE_SCAN_C = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]
Q_AFTER_GRASP_C = [[0.5694903668518067, -1.170427339819336, -0.9575875057434082, 1.3602574621032715, -1.1616069502990722, -1.4733885450805666, 1.5428011756530762], 
[0.17372332402954102, -0.7102331039794922, -0.39615053801879885, 2.2794954481933596, -1.0415729537841798, -1.570412829803467, -0.03873301484985352],
[0.5368932751464844, -0.0015339807861328126, -0.17257283843994142, 1.6014759407226564, -1.8588012175964357, -1.570412829803467, -0.03911651004638672],
[0.8920098271362306, 0.10737865502929689, 0.08206797205810547, 1.6866118743530274, -0.21053886289672852, -0.5725583284240723, -0.03834951965332031]
]

################################################################################################
Q_SCAN_BIN_D = [1.0561457712524416, -0.8613302114135742, -0.15454856420288088, 0.9568205153503418, 0.09012137118530274, 1.597257493560791, 0.21552430045166018]
Q_SCAN_BIN_D_HAYDEN = [1.4028254289184572, -0.4571262742675782, -1.0285341171020508, 0.5234709432678223, 0.8716845817199708, 1.7142235285034182, -0.03067961572265625]
Q_IK_SEED_D = [0.4421699616027832, -1.2264176385131837, 0.5875146410888672, 1.7740487791625978, 0.12540292926635743, -0.4720825869323731, 0.0]

Q_AFTER_SCAN_D = [

[0.6408204734069824, -0.8571117642517091, -0.042184471618652346, 2.408349834228516, -0.05560680349731446, -0.09664078952636719, 0.18024274237060547],
[0.21514080525512697, -1.2302525904785158, 1.3173060000915529, 2.2856313713378906, 0.5541505589904786, -1.1236409258422853, -0.5349757991638184]
]
Q_BEFORE_SCAN_D = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]

Q_AFTER_GRASP_D = [
[0.21514080525512697, -1.2302525904785158, 1.3173060000915529, 2.2856313713378906, 0.5541505589904786, -1.1236409258422853, -0.5349757991638184],
[-0.13920875634155275, -1.310403086553955, 1.1708108350158692, 2.4589712001708985, -1.038121497015381, -0.9675583808532715, 0.5042961834411621],
[0.6408204734069824, -0.8571117642517091, -0.042184471618652346, 2.408349834228516, -0.05560680349731446, -0.09664078952636719, 0.18024274237060547]
]

################################################################################################
Q_SCAN_BIN_E = [0.8582622498413086, -1.0009224629516602, -0.2201262428100586, 1.2268011337097169, 0.07209709694824219, 1.3552720245483398, -0.2676796471801758]
Q_SCAN_BIN_E_HAYDEN = [0.7574030131530762, -0.37007286465454103, 0.09127185677490235, 0.026077673364257814, -0.09549030393676758, 1.9182429730590822, -0.01112136069946289]
Q_IK_SEED_E = [0.0855194288269043, -1.340315711883545, 0.7090826183898926, 1.9427866656372073, 0.24045148822631837, -0.417242773828125, 0.0]
Q_AFTER_SCAN_E = [[-0.2726650847351074, -1.3986069817565918, 0.9970875109863282, 2.3289663285461426, -0.12770390044555666, 0.8471408891418457, 0.018407769433593752],
[-0.19519905503540042, -1.3886361066467285, 1.0530778096801758, 2.461655666546631, -0.0613592314453125, -1.1424321904724122, -0.04371845240478516]]
Q_BEFORE_SCAN_E = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]

Q_AFTER_GRASP_E = [[-0.19519905503540042, -1.3886361066467285, 1.0530778096801758, 2.461655666546631, -0.0613592314453125, -1.1424321904724122, -0.04371845240478516],
[-0.0011504855895996095, -1.353738043762207, 0.904281673425293, 2.5866751006164552, -0.030296120526123047, -0.029529130133056643, 0.08628641921997071]
]

################################################################################################
# shortcutting may be possible
Q_SCAN_BIN_F = [0.6419709589965821, -1.078004997454834, -0.2933738253479004, 1.3782817363403321, 0.015339807861328126, 1.2256506481201173, -0.15301458341674806]
Q_SCAN_BIN_F_HAYDEN =  [0.5867476506958008, -0.3140825659606934, -0.17525730481567384, 0.3758252926025391, -1.4787574778320314, 1.2064758882934572, -0.04486893799438477]
Q_IK_SEED_F = [0.005368932751464844, -1.1757962725708009, 0.5292233712158203, 1.6996507110351564, -0.0602087458557129, -0.4402524856201172, 0.0]
Q_AFTER_SCAN_F = [[-0.06596117380371094, -1.3702283372131348, 0.5065971546203614, 2.4037478918701174, -0.21053886289672852, 0.5786942515686035, -0.16375244891967775], 
[-0.20056798778686524, -1.3901700874328615, 0.5081311354064941, 2.2499663180603027, -0.6201117327941895, -1.0350535354431154, 0.3819612157470703]
]
Q_BEFORE_SCAN_F = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]
Q_AFTER_GRASP_F = [[-0.20056798778686524, -1.3901700874328615, 0.5081311354064941, 2.2499663180603027, -0.6201117327941895, -1.0350535354431154, 0.3819612157470703],
[-0.061742726641845706, -1.3541215389587404, 0.4705486061462403, 2.5847576246337893, -0.21207284368286133, -0.1817767231567383, -0.120800986907959],
[0.9748447895874024, -0.9073496349975586, 0.23700003145751955, 2.4716265416564944, 0.2323980890991211, -0.28493693102416995, -0.03451456768798828]
]

###############################################################################################
Q_SCAN_BIN_G = [1.1976554987731935, -0.7267233974304199, -0.2937573205444336, 1.2540292926635743, 0.26806314237670903, 1.2183642393859864, 0.33018936421508793]
Q_SCAN_BIN_G_HAYDEN = [1.1508690847961427, -0.2193592524169922, -0.13499030917968752, 0.43526704806518557, 0.1683543912780762, 1.329194351184082, 0.24927187774658205]
Q_IK_SEED_G = [0.7566360227600099, -0.8302671004943848, 0.32827188823242187, 1.7598594568908692, 0.1468786602722168, -0.808024379095459, 0.0]
Q_AFTER_SCAN_G = [[1.1036991756225587, -0.8214467109741211, 0.22281070918579102, 2.2327090342163087, 0.33210684019775394, -0.08935438079223633, -0.1852281799255371],
[0.9384127459167481, -0.9491506114196778, 0.34744664805908204, 2.233476024609375, 0.3804272349609375, -1.2962137642822267, -0.18292720874633792]]

Q_BEFORE_SCAN_G = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]
]
Q_AFTER_GRASP_G = [[0.9384127459167481, -0.9491506114196778, 0.34744664805908204, 2.233476024609375, 0.3804272349609375, -1.2962137642822267, -0.18292720874633792],
[1.1593059791198732, -0.634301055065918, 0.11888351092529298, 2.397611968725586, 0.31906800351562503, -0.7351602917541504, -0.04256796681518555]
]

################################################################################################
Q_BEFORE_SCAN_H = []
Q_SCAN_BIN_H = [1.0565292664489747, -0.8490583651245118, -0.41494180264892583, 1.498699228051758, 0.30564567163696293, 1.1662088926574707, -0.11773302533569337]
Q_SCAN_BIN_H_HAYDEN = [0.8789709904541017, -0.9311263371826173, -0.1342233187866211, 1.5167235022888184, 0.11965050131835939, 1.1619904454956056, 0.0]
Q_IK_SEED_H = [0.9019807022460938, -0.8348690428527833, -0.09242234236450196, 2.0056798778686526, 0.04525243319091797, -1.2279516192993165, 0.0]
Q_AFTER_SCAN_H = [[0.5740923092102052, -1.1692768542297365, 0.12923788123168947, 2.5866751006164552, -0.06941263057250976, 0.02185922620239258, 0.0],
[0.22933012752685547, -1.3019661922302248, 0.45674277907104494, 2.476611979211426, -0.21629129084472656, -1.0653496559692384, 0.0]]

Q_AFTER_GRASP_H = [[0.22933012752685547, -1.3019661922302248, 0.45674277907104494, 2.476611979211426, -0.21629129084472656, -1.0653496559692384, 0.0],
[0.9637234288879395, -0.6787864978637695, 0.027611654150390626, 2.440946925933838, -0.17257283843994142, -0.732475825378418, 0.04755340437011719]
]

################################################################################################
Q_SCAN_BIN_I = [0.5691068716552735, -0.7850146673034668, -0.20401944455566406, 1.3832671738952638, 0.07094661135864258, 1.0415729537841798, -0.13192234760742189]
Q_SCAN_BIN_I_HAYDEN = [0.5848301747131348, -0.8410049659973146, -0.10622816943969728, 1.4622671843811037, 0.010737865502929688, 1.0327525642639162, 0.0]
Q_IK_SEED_I = [0.19328157905273438, -0.681470964239502, 0.2554078008911133, 1.6816264367980958, -0.2059369205383301, -0.9644904192810059, 0.0]
Q_AFTER_SCAN_I = [[0.1806262375671387, -0.9901845974487306, -0.16682041049194338, 2.421772166107178, -0.7685243738525391, -0.06442719301757813, 0.0],
[0.15033011704101565, -1.0714855791137696, -0.11274758778076173, 2.5659663600036624, -0.5963350306091308, -1.5366652525085451, 0.0]]

Q_BEFORE_SCAN_I = []

Q_AFTER_GRASP_I = [[0.15033011704101565, -1.0714855791137696, -0.11274758778076173, 2.5659663600036624, -0.5963350306091308, -1.5366652525085451, 0.0],
[0.9637234288879395, -0.6787864978637695, 0.027611654150390626, 2.440946925933838, -0.17257283843994142, -0.732475825378418, 0.04755340437011719]
]

################################################################################################
Q_SCAN_BIN_J = [1.3989904769531252, -0.3125485851745606, -0.49585928911743166, 1.1167380123046875, 0.6891408681701661, 1.1336118009521485, 0.07554855371704101]
Q_SCAN_BIN_J_HAYDEN = [1.0143447948303224, 0.25118935372924805, 0.37083985504760747, 0.21207284368286133, -0.32328645067749023, 1.208393364276123, 0.14419419389648439]
Q_IK_SEED_J = [1.0174127564025879, 0.06327670742797852, 0.11888351092529298, 1.0112768332580566, 0.16451943931274415, -1.2225826865478517, 0.0]
Q_AFTER_SCAN_J = [[1.508286607965088, -0.4007524803771973, -0.08321845764770508, 2.1794032018981935, 0.5928835738403321, -0.4287476297241211, 0.10507768385009766],
[1.5182574830749513, -0.4252961729553223, -0.03873301484985352, 2.1479565957824707, 0.6139758096496583, -1.5692623442138673, 0.09127185677490235]
]
Q_BEFORE_SCAN_J = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]

Q_AFTER_GRASP_J = [[1.5182574830749513, -0.4252961729553223, -0.03873301484985352, 2.1479565957824707, 0.6139758096496583, -1.5692623442138673, 0.09127185677490235],
[1.3759807651611329, -0.38349519653320313, -0.05714078428344727, 2.1985779617248538, 0.32060198430175785, -0.6381360070312501, 0.1307718620178223]
]

################################################################################################
Q_SCAN_BIN_K = [1.034670040246582, -0.3857961677124024, -0.36930587426147465, 1.330728331970215, 0.3742913118164063, 0.8774370096679688, -0.14841264105834961]
Q_SCAN_BIN_K_HAYDEN = [0.8057234079162598, 0.17985924717407228, -0.0609757362487793, 0.37390781661987305, -0.0015339807861328126, 1.0166457660095216, -0.016106798254394532]
Q_IK_SEED_K = [0.568339881262207, -0.015723303057861328, 0.19059711267700197, 1.2613157013977052, -0.16566992490234375, -1.3069516297851564, 0.0]
Q_AFTER_SCAN_K = [[0.822980691760254, -1.0285341171020508, -0.07401457293090821, 2.4052818726562504, -0.034898062884521484, 0.2653786760009766, -0.02531068297119141],
[0.7819467057312012, -0.6914418393493653, -0.04601942358398438, 2.1847721346496582, -0.04026699563598633, -1.2570972542358398, -0.027228158953857422]]
Q_BEFORE_SCAN_K = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237]]

Q_AFTER_GRASP_K = [[0.7819467057312012, -0.6914418393493653, -0.04601942358398438, 2.1847721346496582, -0.04026699563598633, -1.2570972542358398, -0.027228158953857422],
[1.3759807651611329, -0.38349519653320313, -0.05714078428344727, 2.1985779617248538, 0.32060198430175785, -0.6381360070312501, 0.1307718620178223]
]

################################################################################################
Q_SCAN_BIN_L = [0.6615292140197754, -0.3497476192382813, -0.32251946028442385, 1.1604564647094728, 0.19328157905273438, 0.9909515878417969, -0.17525730481567384]
Q_SCAN_BIN_L_HAYDEN = [-0.08858739039916992, 0.42069423059692385, 1.285475898779297, 0.8187622445983888, -1.8158497555847168, 1.0561457712524416, -0.09012137118530274]
Q_IK_SEED_L = [0.09165535197143555, -0.004218447161865235, 0.27151459914550785, 1.1316943249694824, -0.5691068716552735, -1.102165194836426, 0.0]
Q_AFTER_SCAN_L = [[0.0847524384338379, -0.25847576246337894, 0.0011504855895996095, 1.7905390726135255, -0.7478156332397461, -0.06250971703491211, -0.10507768385009766],
[0.08666991441650392, -0.29682528211669923, 0.08782040000610353, 1.7598594568908692, -0.7623884507080079, -1.4963982568725587, -0.10431069345703126]
]

Q_BEFORE_SCAN_L = [[0.8839564280090333, -0.7539515563842774, -0.1817767231567383, 2.2756604962280274, 0.1606844873474121, 0.16221846813354493, -0.18944662708740237],
[0.6876068873840332, -0.22396119477539064, -0.033364082098388675, 1.864937140740967, -0.3175340227294922, -0.26116022883911133, 0.1100631214050293]
]

Q_AFTER_GRASP_L = [[0.08666991441650392, -0.29682528211669923, 0.08782040000610353, 1.7598594568908692, -0.7623884507080079, -1.4963982568725587, -0.10431069345703126],[1.3759807651611329, -0.38349519653320313, -0.05714078428344727, 2.1985779617248538, 0.32060198430175785, -0.6381360070312501, 0.1307718620178223]
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
VACUUM_POINT_XFORM = (so3.identity(), [.07, 0.02, .38])

# Local transform from right_lower_forearm to F200
RIGHT_F200_CAMERA_CALIBRATED_XFORM = ([0.029405922588322576, 0.01584833204831371, -0.9994419053091667, 0.9885099606860334, 0.14785225126913087, 0.031428799199895927, 0.14826782975826966, -0.9888824713614545, -0.011318502234688471], [0.11999999999999998, -0.11, 0.10500000000000001])

# Transform of shelf relative to world
SHELF_MODEL_XFORM = [1.43,-.23,-.02]

# Distances (meters)
GRASP_MOVE_DISTANCE = .05
BACK_UP_DISTANCE = .2

# Times (seconds)
MOVE_TIME = 2
SCAN_WAIT_TIME = 0 if REAL_PERCEPTION else 0
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
