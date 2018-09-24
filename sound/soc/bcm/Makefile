# BCM2835 Platform Support
snd-soc-bcm2835-i2s-objs := bcm2835-i2s.o

obj-$(CONFIG_SND_BCM2835_SOC_I2S) += snd-soc-bcm2835-i2s.o

# CYGNUS Platform Support
obj-$(CONFIG_SND_SOC_CYGNUS) += cygnus-pcm.o cygnus-ssp.o

obj-$(CONFIG_SND_SOC_CYGNUS_AUD_BASE_MACHINE) += bcm958305_aud_base-machine.o bcm-card-utils.o cygnus-clk-utils.o
obj-$(CONFIG_SND_SOC_CYGNUS_SVK_MACHINE) += cygnus_svk-machine.o
obj-$(CONFIG_SND_SOC_CYGNUS_VOIPPHONE_MACHINE) += bcm911360_entphn-machine.o bcm-card-utils.o

# Omega Platform Support
obj-$(CONFIG_SND_SOC_OMEGA) += cygnus-pcm.o omega-ssp.o

obj-$(CONFIG_SND_SOC_OMEGA_BCM91140X15_ENTPHN_MACHINE) += bcm91140x15-entphn-machine.o bcm-card-utils.o
obj-$(CONFIG_SND_SOC_OMEGA_BCM958313_WA_MACHINE) += bcm958313_wa-machine.o bcm-card-utils.o
obj-$(CONFIG_SND_SOC_OMEGA_SVK_MACHINE) += omega_svk-machine.o bcm-card-utils.o
