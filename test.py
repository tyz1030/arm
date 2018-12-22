import csv


"""config = ConfigParser.ConfigParser()
config.read("//util//calibration.cfg")

intrinsic_matrix = config.get("section1","intrinsic matrix")
distortion_coefficient = config.get("section1","distortion coefficient")

#f = open("//util//calibration.cfg")
"""

#intrinsic_matrix = [[522.37722154,0,306.67215671],[0,523.85826269,239.12519563],[0,0,1]]
"""intrinsic_matrix.append([522.37722154,0,306.67215671])
intrinsic_matrix.append([0,523.85826269,239.12519563])
intrinsic_matrix.append([0,0,1])"""
#distort_coef = [0.25053573,-0.82260761,-0.0017132,-0.00132055,0.92750223]

with open("util/calibration.csv", 'rb') as csvfile:
    csvreader = csv.reader(csvfile, delimiter=",", quotechar="|")
    tmp = []
    intrinsic_matrix = []
    distort_coef = []
    i = 0
    for row in csvreader:
        for col in row:
            try:
                tmp.append(float(col))
            except:
                print("ERROR in calibration.csv intrinsic matrix")
        if(i!=3):
            intrinsic_matrix.append(tmp)
            i += 1
            tmp = []
        if(i==3):
        	distort_coef = tmp
        	tmp = []
   
print(intrinsic_matrix)
print(distort_coef)


