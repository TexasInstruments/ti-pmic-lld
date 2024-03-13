import os
import subprocess

if __name__ == "__main__":
    kwlpFile = ".kwlp"
    kwpsFile = ".kwps"
    pconfFile = "analysis_profile_SA_plus_MISRAC_2012_HIS.pconf"
    mconfFile = "his_metrics_community.mconf"
    sconfFile = "kw_filter.sconf"

    if ((not os.path.exists(kwlpFile)) and (not os.path.exists(kwpsFile))):
        subprocess.run(["kwcheck", "create"])
        subprocess.run(["kwcheck", "set", "license.host=kw-lic.ent.ti.com", "license.port=27005"])
        if (os.path.exists(pconfFile)):
            subprocess.run(["kwcheck", "import", pconfFile])
        else:
            print(pconfFile + " not found.")
            exit(-1)
        if (os.path.exists(mconfFile)):
            subprocess.run(["kwcheck", "import", mconfFile])
        else:
            print(mconfFile + " not found.")
            exit(-1)
        if (os.path.exists(sconfFile)):
            subprocess.run(["kwcheck", "import", sconfFile])
        else:
            print(sconfFile + " not found.")
            exit(-1)
        