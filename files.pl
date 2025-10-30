:- portray_text(true).    % Enable portray of strings

:- multifile user:file_search_path/2.
:- dynamic user:file_search_path/2.

user:file_search_path(src, './src').

:- use_module(src(flightgear)).
:- use_module(src(spi)).
