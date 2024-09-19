Import("env")
import os, shutil

targetdir = "%s" % env.GetProjectOption("custom_fw_name")
print("Target is", targetdir)
progname = "%s_%s.bin" % (env.GetProjectOption("custom_fw_name"), env.GetProjectOption("custom_fw_version"))

def afterBuild(target, source, env):
    print("We are at ", os.getcwd())
    print("Target: ", target)
    print("Source: ", source)
    print("Env: ", env)
    print("Progname:", progname)
    srcpath = ".pio/build/%s/%s" % (targetdir, progname)
    dstpath = progname
    print("Copying", srcpath, "to", dstpath)
    shutil.copy(srcpath, dstpath)

env.AddPostAction("buildprog", afterBuild)