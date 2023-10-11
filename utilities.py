from math import atan2, asin

M_PI=3.1415926535

class Logger:
    """
    Used for logging sensor data
    """

    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        self.filename = filename

        with open(self.filename, 'w') as file:
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):

        with open(self.filename, 'a') as file:
            vals_str = ""
            vals_str += str(values_list).replace("[", "").replace("]", "")
            vals_str += "\n"
            
            file.write(vals_str)
            

    def save_log(self):
        pass

class FileReader:
    """
    Extracts data from CSV files into lists
    """
    def __init__(self, filename):
        
        self.filename = filename
        
        
    def read_file(self):
        
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # Skip the header line

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val == '':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table


def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    # TODO: pitch and roll should be confirmed
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    yaw = atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
    pitch = atan2(2 * (w * x + y * z), 1 - 2 * (x ** 2 + y ** 2))
    roll = asin(2 * (w * y - x * z))
    return yaw
