import sys
import pandas as pd
import os
import glob


def main():
    loc = os.getcwd()
    print("loc", loc)
    files = glob.glob(loc + "/data_v1/*")

    for f in files:
        print(f)
        os.remove(f)
    print("reading path", loc +"/data/" + "para.csv")
    try:
        paramater = pd.read_csv(loc + "/data/" + "para.csv")
        print("reading paramater.xlsx", paramater)
        print(paramater.dtypes)
    except Exception:
        print("error opening paramater.xlsx")

    valid_input = []
    for ind, row in paramater.iterrows():
        readfile = row["data_set"]
        for filename in os.listdir(loc + "/data"):
            if filename.endswith(".xlsx"):
                continue
            if str(readfile) == filename[: len(filename) - 4]:
                df = pd.read_csv(loc + "/data/" + filename)

                print("match", readfile)

                # Number of customer,data_set,time_min,sum_low,sum_upper,num_truck,num_drone,working_time,truck_capacity,drone_capacity,drone_velocity,truck_velocity,drone_duration
                number_truck = row["num_truck"]
                number_drone = row["num_drone"]
                working_time = row["working_time"]
                truck_capacity = row["truck_capacity"]
                drone_capacity = row["drone_capacity"]
                drone_speed = row["drone_velocity"]
                truck_speed = row["truck_velocity"]
                drone_duration = row["drone_duration"]
                # try to get paramater from paramater.xlsx
                try:
                    with open(
                        loc + "/data_v1/" + filename[: len(filename) - 3] + "txt",
                        "w",
                    ) as outfile:
                        try:
                            print(
                                (number_truck),
                                (number_drone),
                                (working_time),
                                end="\n",
                                file=outfile,
                            )
                            print(
                                (truck_speed),
                                (drone_speed),
                                int(truck_capacity),
                                int(drone_capacity),
                                int(drone_duration),
                                end="\n",
                                file=outfile,
                            )
                        except:
                            pass

                        for ind, cus in df.iterrows():
                            try:
                                print(
                                    cus["x"],
                                    cus["y"],
                                    int(cus["low"]),
                                    int(cus["upper"]),
                                    int(cus["weight"]),
                                    end="\n",
                                    file=outfile,
                                )
                            except:
                                for val in cus:
                                    print(int(val), end=" ", file=outfile)
                                print("", file=outfile)
                        valid_input.append(filename[: len(filename) - 4])
                except:
                    print("error value type", filename)
                break


if __name__ == "__main__":
    main()
