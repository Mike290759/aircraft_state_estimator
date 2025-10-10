:- portray_text(true).    % Enable portray of strings
:- working_directory(WD, WD), format(atom(VE), '~wflightgear', [WD]), setenv('VIRTUAL_ENV',  VE).    % Make Janus use the Python virtual environment containing FlightGear
:- py_add_lib_dir('./src').

:- multifile user:file_search_path/2.
:- dynamic user:file_search_path/2.

user:file_search_path(src, './src').

:- use_module(src(flightgear)).
