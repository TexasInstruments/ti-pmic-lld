import os

if __name__ == "__main__":
    kwReport = "kw_report.xml"
    pathToDriver = "C:\\Users\\a0500030\\workspace_v12\\Burton_RTOS_Driver_Workspace\\"
    
    # As we are about to load the report in memory, 
    # ensure that the report is not over 10 MB
    try: 
        if (os.path.getsize(kwReport) > 10000000):
            print("kw_script error: file size greater than 10 MB")
            exit(-1)
    except:
        print("kw_script_error: kw_report.xml does not exist")
        exit(-2)

    # Overwrite first line of KW report to change the encoding;
    # Delete any occurances of the path to pmic_drv
    with open(kwReport, 'r') as file:
        data = file.readlines()
    data[0] = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
    for i in range (1, len(data)):
        data[i] = data[i].replace(pathToDriver, "")
    with open(kwReport, 'w') as file:
        file.writelines(data)
