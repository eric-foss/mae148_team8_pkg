import pyproj 

def LonLat_To_XY(Lon, Lat):
    P = pyproj.Proj(proj='utm', zone=11, ellps='WGS84', preserve_units=True)
    return P(Lon, Lat)    

def XY_To_LonLat(x,y):
    P = pyproj.Proj(proj='utm', zone=11, ellps='WGS84', preserve_units=True)
    return P(x, y, inverse=True)    

def distance(Lat1, Lon1, Lat2, Lon2):
    G = pyproj.Geod(ellps='WGS84')
    return G.inv(Lon1, Lat1, Lon2, Lat2)[2]

def initialize_wksp_obs(filename):
    

    with open(filename) as data:
        coordsdict={}
        name=''
        long=0
        lat=0
        alt=0
        coordslist=[]
        ang=0
        obs_dict={}
        for line in data:
            if '-----' in line:
                if 'obs' in name:
                    obs_dict.update({name:coordslist})                  
                else:
                    coordsdict.update({name:coordslist})
                coordslist=[]
                
            elif '#' in line:
                name=line[1:len(line)-1]
            elif 'latitude' in line:
                lat=float(line[10:])
            elif 'longitude' in line:
                long=float(line[11:])
            elif 'end' in line:
                coordsdict.update({"obs_dict":obs_dict})
            elif line=="\n":
                temp=LonLat_To_XY(long,lat)
                if name=='origin':
                    x_origin=temp[0]
                    y_origin=temp[1]
                    coordslist.append(temp)
                else:
                    x=temp[0]-x_origin
                    y=temp[1]-y_origin
                    coordslist.append((x,y))
    return coordsdict

if __name__ == '__main__':
    initialize_wksp_obs('test2.txt')
    print