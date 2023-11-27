import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from utilities import FileReader

def plot_errors():
    img = mpimg.imread("room.pgm")
    imgplot = plt.imshow(img, cmap="gray")
    plt.show()
    # headers, values=FileReader("robot_pose.csv").read_file()

    
    # time_list=[]
    
    # first_stamp=values[0][-1]
    
    # for val in values:
    #     time_list.append(val[-1] - first_stamp)



    # for i in range(0, len(headers) - 1):
    #     plt.plot(time_list, [lin[i] for lin in values], label= headers[i]+ " linear")

    # plt.legend()
    # plt.grid()

    # plt.show()
    
    

if __name__=="__main__":
    plot_errors()