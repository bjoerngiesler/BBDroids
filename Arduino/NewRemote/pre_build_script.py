Import("env")

env.Replace(PROGNAME="%s_%s" % (env.GetProjectOption("custom_fw_name"), env.GetProjectOption("custom_fw_version")))