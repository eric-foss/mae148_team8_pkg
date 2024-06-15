import pyproj 
from mae148_team8_pkg.path import CTE
from transform import PIDController
#ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

def getzangrot(pathcoords,currPoint,pid_class,cte_class):
    ##input
    #pathcoords is ordered list of tuple pairs (x,y) defining path from 
    #start to end
    #currPoint is tuple (x,y) of current gps coord with reference to origin
    #steering is a value -1 to 1 for z ang rot

    cte,i=cte_class.run(pathcoords,currPoint[0],currPoint[1])

    alpha=pid_class.run(cte)

    if alpha>1:
        steering=1
    elif alpha<-1:
        steering=-1
    else:
        steering=alpha

    return steering



# if __name__ == '__main__':
#     path=[(-35.0, 0.0), (-32.7477979597946, 15.707305500128614), (92.93231285047175, 19.195626260247085), (89.26903370620371, 8.343862239737062), (90.0, 7.0)]
#     currPoint=(-34.5,7)

#     path=[(0.0, 0.0), (0, 1), (0, 3), (0, 4), (0, 5)]
#     path=[(0.18499525036895648, -0.23691183887422085), (0.2609459265950136, -0.4347797315567732), (0.2292255109641701, -0.6767509416677058), (0.2567807122832164, -0.9040732267312706), (0.29829285287996754, -1.1646843324415386), (0.3444655549246818, -1.4326964323408902), (0.37986531772185117, -1.6378644364885986), (0.41369781736284494, -1.8467242401093245), (0.4490728444652632, -2.0629780809395015), (0.48598627879982814, -2.2884735967963934), (0.5291522707557306, -2.50659246975556), (0.5598420067690313, -2.726531152613461), (0.5858629387221299, -2.9427641197107732), (0.6165815386339091, -3.149769325274974), (0.6378718271153048, -3.390011171810329), (0.6668623803416267, -3.6727661197073758), (0.7020642699790187, -3.966620820108801), (0.6936260472866707, -4.254835199564695), (0.6431026029167697, -4.539260375779122), (0.6393455525394529, -4.82563755242154), (0.6527376922313124, -5.112052990589291), (0.6661669368040748, -5.381839676294476), (0.6795384671422653, -5.6774933147244155)]
#     currPoint=(-1,-1)
#     P=0.075
#     I=0.01
#     D=0.15 
#     PID=PIDController(P,I,D)
#     look_ahead=1
#     look_behind=0
#     CTE=CTE(look_ahead,look_behind)
#     print(getzangrot(path,currPoint,PID,CTE))

