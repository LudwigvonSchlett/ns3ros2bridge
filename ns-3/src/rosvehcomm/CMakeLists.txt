build_lib(
	LIBNAME rosvehcomm
	SOURCE_FILES
		model/rosvehicule.cc
		model/rosvehsync.cc
    	helper/rosvehcomm-helper.cc
    HEADER_FILES
    	model/rosvehicule.h
		model/rosvehsync.h
    	helper/rosvehcomm-helper.h
    LIBRARIES_TO_LINK 	${libcore}
    					${libwifi}
    					${libapplications}
    					${libmobility}
    					${libinternet}
    					${libconfig-store}
    					${libstats}
    					${libfd-net-device}
    					${libnetanim}
    TEST_SOURCES
    	test/rosvehcomm-test-suite.cc
)
