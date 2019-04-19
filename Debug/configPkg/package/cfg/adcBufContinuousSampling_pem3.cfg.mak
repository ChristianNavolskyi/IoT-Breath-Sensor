# invoke SourceDir generated makefile for adcBufContinuousSampling.pem3
adcBufContinuousSampling.pem3: .libraries,adcBufContinuousSampling.pem3
.libraries,adcBufContinuousSampling.pem3: package/cfg/adcBufContinuousSampling_pem3.xdl
	$(MAKE) -f C:\Users\user\workspace_v9\rfPacketTx_CC1350_LAUNCHXL_TI/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\user\workspace_v9\rfPacketTx_CC1350_LAUNCHXL_TI/src/makefile.libs clean

