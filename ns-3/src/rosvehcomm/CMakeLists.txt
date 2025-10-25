build_lib(
	LIBNAME rosvehcomm
	SOURCE_FILES
		model/rosvehcomm.cc
    	helper/rosvehcomm-helper.cc
    HEADER_FILES
    	model/rosvehcomm.h
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
