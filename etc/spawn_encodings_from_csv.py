import pandas as pd


def filter_comm_flag(df: pd.DataFrame, comm_allowed: int):
    ind = df[df['CommunicationAllowed'] != comm_allowed].index
    return df.drop(ind)

def filter_agent_count(df: pd.DataFrame, agent_count : int):
    ind = df[df['AgentCount'] != agent_count].index
    return df.drop(ind)



if __name__ == '__main__':

    df = pd.read_csv("/home/lisa/wumpusws/worlds/pg5/Results_all.csv", sep=",")
    target_directory = "/home/lisa/wumpusws/worlds/pg5/"

    df = filter_comm_flag(df, 0)

    pg_size = 5

    num_fields = pg_size * pg_size

    # create 5 files with spawn position encodings from runs without communication

    for i in range(2,6): #2,3,4,5
        # 2 agents: 200 - 300, 3 agents: 300 - 400
        part = filter_agent_count(df, i).reset_index()
        print(part)
        cols = ["StartPointAgent{}".format(k+1) for k in range(0, i)]
        encodings = list()

        for j in range(len(part)):
            current_encoding = "0" * num_fields
            for c in cols:

                print("Col:{}".format(c))
                start_point  = part[c][j]
                coord_x = start_point[start_point.find("(") + 1 : start_point.find("-")]
                coord_y = start_point[start_point.find("-") + 1 : start_point.find(")")]

                field_index = pg_size * int(coord_y) + int(coord_x)
                current_encoding = current_encoding[:field_index] + "1" + current_encoding[field_index + 1:]


                if(len(current_encoding) != num_fields):
                    raise ValueError("Wrong length for encoding!")
            print("Current Encodign:{}".format(current_encoding))

            encodings.append(current_encoding)

        enc = pd.DataFrame(data=encodings,columns=["Encoding"])
        enc.to_csv("{}pg{}Encodings{}".format(target_directory,pg_size,i), header=False,index=False)