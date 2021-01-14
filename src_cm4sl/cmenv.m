function cmenv (varargin)		% -*- Mode: Fundamental -*-
% CMENV - Add CarMaker directories to the MATLAB search path.
%

    % CarMaker installation directory.
    if isempty(which('cmlocaldir'))
	cminstdir = '/opt/ipg/carmaker/linux64-9.1';
    else
	cminstdir = cmlocaldir	% for mat: CM-9.0
    end

    disp(['CarMaker directory: ', cminstdir]);
    if ~exist(cminstdir, 'dir')
	error('Unable to find specified CarMaker installation directory.');
    end

%    if is_64_bit_version
%	unsupported_version_error(cminstdir);
%    end

    bestreldir = match_matlab_release(cminstdir);
    if isempty(bestreldir)
	unsupported_version_error(cminstdir);
    end

    % Remove all CarMaker directories from search path before adding new ones.
    if ~isempty(which('cmcmd'))
	engrunning = cmcmd('engrunning');
    else
	engrunning = 0;
    end
    if engrunning
	% If CM4SL engine is running, cleanup_path() would badly interfere with it.
	cmcmd('engstop');
    end
    cleanup_path();

    % Add path to CarMaker's Matlab Support stuff (indep. of Matlab version).
    p = fullfile(cminstdir, 'Matlab');
    try_addpath(p);

    % Add path to CarMaker's Matlab Support stuff (Matlab version specific).
    p = fullfile(cminstdir, 'Matlab', bestreldir);
    try_addpath(p);

    % Add path to MotorcycleMaker for Simulink stuff.
    p = fullfile(cminstdir, 'MM4SL');
    if try_addpath(p)
	% Add path to MotorcycleMaker for Simulink stuff (Matlab version specific).
	p = fullfile(cminstdir, 'MM4SL', bestreldir);
	try_addpath(p);
    end

    % Add path to TruckMaker for Simulink stuff.
    p = fullfile(cminstdir, 'TM4SL');
    if try_addpath(p)
	% Add path to TruckMaker for Simulink stuff (Matlab version specific).
	p = fullfile(cminstdir, 'TM4SL', bestreldir);
	try_addpath(p);
    end

    % Add path to CarMaker for Simulink stuff.
    p = fullfile(cminstdir, 'CM4SL');
    if try_addpath(p)
	% Add path to CarMaker for Simulink stuff (Matlab version specific).
	p = fullfile(cminstdir, 'CM4SL', bestreldir);
	try_addpath(p);

	% Add path to the user's own compiled version of the CarMaker library.
	try_addpath(fullfile(dirname(pwd), 'src_cm4sl'));
	try_addpath(fullfile(dirname(pwd), 'src_mm4sl'));
	try_addpath(fullfile(dirname(pwd), 'src_tm4sl'));

	if engrunning
	    cmcmd('engstart');
	else
	    cminit;
	end
    end


function bestreldir = match_matlab_release (cminstdir)
% Search cminstdir for the directory best matching the current Matlab version.
% Currently an exact version match is required.

    thisrel = ['R', version('-release')];
    [v,n] = sscanf(version, '%d.%d.%d.%d');
    if v(3) > 0
	thisrel = sprintf('%sSP%d', thisrel, v(3));
    end

    bestreldir = '';

    matreldirlist = dir(fullfile(cminstdir, 'Matlab'));
    for i=1:length(matreldirlist)
	if ~matreldirlist(i).isdir
	    continue
	end
	reldir = matreldirlist(i).name;
	if strcmp(lower(reldir), lower(thisrel))
	    bestreldir = reldir;
	    break
	end
    end


function cleanup_path ()
% Remove all search path entries of other CarMaker versions.

    if isunix
	arch   = 'linux';
	arch2  = 'linux64';
	dirsep = '/';
    else
	arch   = 'win32';
	arch2  = 'win64';
	dirsep = '\';
    end
    fragment  = sprintf('%shil%s%s-',      dirsep, dirsep, arch);
    fragment2 = sprintf('%scarmaker%s%s-', dirsep, dirsep, arch2);

    p = path;
    while ~isempty(p)
	[d, p] = strtok(p, pathsep);
	if ~isempty(strfind(d, fragment)) || ~isempty(strfind(d, fragment2))
	    % disp(sprintf('rmpath %s', d));
	    rmpath(d);
	end
    end


function b = try_addpath (p)
% If p exists, add it to the MATLAB search path. 

    if exist(p, 'dir')
	disp(sprintf('addpath %s', p));
	addpath(p);
	b = 1;
    else
	b = 0;
    end


function parent = dirname (p)
% Return parent directory of p.

    [parent,n,e] = fileparts(p);


function b = is_64_bit_version ()
% Return whether a 64-bit version of Matlab is running.

    b = ~isempty(strfind(computer, '64'));


function nv = numver ()
% Return matlab version as a number suitable for easy version comparison.
% Examples: R13 = 6.5 = 60500, R14SP3 = 7.1 = 70100, R2007b = 7.5 = 70500, etc.
    v = sscanf(version, '%d.%d.%d');
    nv = 100*(100*v(1) + v(2)) + v(3);


function unsupported_version_error (cminstdir)
% Issue an error message stating incompatibility with the running Matlab version.

    if is_64_bit_version
	nbits = 64;
    else
	nbits = 32;
    end

    msg = sprintf('\nThis CarMaker version does not support %d-bit Matlab R%s.\nSee the CarMaker release notes for a list of supported Matlab versions.', nbits, version('-release'));

    errordlg(msg, 'CarMaker Error');
    error(msg);

