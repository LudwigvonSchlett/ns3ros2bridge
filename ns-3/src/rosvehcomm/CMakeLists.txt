build_lib(
	LIBNAME rosvehcomm
	SOURCE_FILES
		model/rosheader.cc
		model/rosvehicule.cc
		model/rosvehsync.cc
    	helper/rosvehsync-helper.cc
		helper/rosvehicule-helper.cc
    HEADER_FILES
		model/rosheader.h
    	model/rosvehicule.h
		model/rosvehsync.h
    	helper/rosvehsync-helper.h
		helper/rosvehicule-helper.h
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
