# invoke SourceDir generated makefile for app_ble.pem3
app_ble.pem3: .libraries,app_ble.pem3
.libraries,app_ble.pem3: package/cfg/app_ble_pem3.xdl
	$(MAKE) -f E:\ccs_work\ble5_hid_gyromouse_app\TOOLS/src/makefile.libs

clean::
	$(MAKE) -f E:\ccs_work\ble5_hid_gyromouse_app\TOOLS/src/makefile.libs clean

