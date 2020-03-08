% to visualize: visu: http://gcode.ws/

filename = 'xyzCalibration_cube_0.15mm_SPEED_PLA_MK3S_52m.gcode';

% distances along path to new points (previous and next) in mm
dtarget = 0.1;  
dMin = 0.02; % minimum distance: segments shorter than 3 * dMin  dont get their corners cut

%%
[filepath,filename0,fileext] = fileparts(filename);
gcode = importgcode(filename);

nLinesOld = size(gcode,1);

%%
isG1 = startsWith(gcode,'G1');
isG0 = startsWith(gcode,'G0') | contains(gcode,'; move');

isG0G1 = isG1 | isG0;
nG0G1 = sum(isG0G1);

linesWithG0G1 = gcode(isG0G1);
XYZEFidx = cumsum(isG0G1);

%%
a = startsWith(gcode,';BEFORE_LAYER_CHANGE');
startline = find(a,1);

nG0G1skip = sum(isG0G1(1:startline));
%%
XYZEFall = nan(nG0G1,5);
identifiers = {'X','Y','Z','E','F'};
for ii = 1:5
    tokens = regexp(linesWithG0G1,[identifiers{ii},'([-\d.]+)'],'tokens');
    hasItem = ~cellfun(@isempty,tokens);
    tokens1 = tokens(hasItem,1);
    % unpack nested cells
    tokens1 = vertcat(tokens1{:});
    tokens1 = vertcat(tokens1{:});
    
    XYZEFall(hasItem,ii) = str2double(tokens1);
end

isXYZEFspecified = ~isnan(XYZEFall);
XYZEFall = fillmissing([zeros(1,5);XYZEFall],'previous',1);
XYZEFall = XYZEFall(2:end,:);

%%
XYZEF = XYZEFall(2:end-1,:);
prevXY = XYZEFall(1:end-2,1:2);
nextXY = XYZEFall(3:end,1:2);

%%
isXMove = isXYZEFspecified(:,1);
isYMove = isXYZEFspecified(:,2);
isG0_ = isG0(isG0G1);

% find active corners to split:
% isXMove or isYMove and not G0
% next move is isXMove or isYMove and not G0

isActive = (isXMove | isYMove) & ~isG0_;
isActive = isActive & [isActive(2:end);0];
isActive(1:nG0G1skip) = false;

% nex movement command where E must be subtracted potentially
subtractELocation = 2:nG0G1+1;
subtractELocation = subtractELocation(isActive);

nActive = sum(isActive);

%%

xy = XYZEFall(isActive,1:2);
xyp = prevXY(isActive(2:end-1),:);
xyn = nextXY(isActive(2:end-1),:);

% direction vectors
vecp = xy - xyp;
vecn = xyn - xy;

lenp = vecnorm(vecp,2,2);
vecpNorm = vecp ./ lenp; % normalized direction vector
lenn = vecnorm(vecn,2,2);
vecnNorm = vecn ./ lenn; % normalized direction vector

extrudePerLengthp = XYZEFall(isActive,4) ./ lenp;
%%

dpTarget = dtarget * ones(nActive,1);
dnTarget = dtarget * ones(nActive,1);

dpMax = lenp/4;
dnMax = lenn/4;

dpMin = dMin * ones(nActive,1);
dnMin = dMin * ones(nActive,1);

dp = min(dpTarget,dpMax);
dp  = max(dp,dpMin);
dn = min(dnTarget,dnMax);
dn  = max(dn,dnMin);

% if the new sections are too small, they are invalid
isValid = dpMin<dpMax & dnMin<dnMax;
dp(~isValid) = 0;
dn(~isValid) = 0;

xyNewp = xy - vecpNorm.* dp;
xyNewn = xy + vecnNorm.* dn;

newLenp = [0; lenp(2:end)- dp(2:end)];
newLenn = vecnorm(xyNewn-xyNewp,2,2);

eNewp = newLenp .* extrudePerLengthp;
eNewn = newLenn .* extrudePerLengthp;

deltaENewpp = dn .* extrudePerLengthp;

XYZEFall(subtractELocation,:) = XYZEFall(subtractELocation,:) - deltaENewpp;

%%

XYZEFallWithNan = XYZEFall;
XYZEFallWithNan(~isXYZEFspecified) = nan;
GcodeG0G1 = compose("G1 X%.3f Y%.3f Z%.3f E%.6f F%i",XYZEFallWithNan);
GcodeG0G1 = erase(GcodeG0G1,{' XNaN',' YNaN',' ZNaN',' ENaN',' FNaN'});

gcode2 = gcode;
gcode2(isG0G1) = GcodeG0G1;

%%
GcodeNewp = compose("G1 X%.3f Y%.3f E%.6f",[xyNewp(isValid,:) eNewp(isValid,:)]);
GcodeNewn = compose("G1 X%.3f Y%.3f E%.6f",[xyNewn(isValid,:) eNewn(isValid,:)]);

%%
% nNewPoints = sum(isValid);
% nOldLines = sum(~useG0G1) + sum(~isInternalVertex);

isReplacedIndices = 1:size(gcode2,1);
isReplacedIndices = isReplacedIndices(isG0G1);
isReplacedIndices = isReplacedIndices(isActive);
isReplacedIndices = isReplacedIndices(isValid);

isReplaced = false(size(gcode2,1),1);
isReplaced(isReplacedIndices) = true;

nNewLinesPerOldLine = 1+isReplaced;
nNewLinesTotal = sum(nNewLinesPerOldLine);

newLineIdx = cumsum(nNewLinesPerOldLine);

%%
newGcode = strings(nNewLinesTotal,1);
newGcode(newLineIdx(~isReplaced)) = gcode2(~isReplaced);

newGcode(newLineIdx(isReplaced)-1) = GcodeNewp;
newGcode(newLineIdx(isReplaced)) = GcodeNewn;

%%
% remove comments
newGcode(startline:end) = regexprep(newGcode(startline:end),';.*','');

% remove empty lines
toRemoveEmpty = strlength(newGcode) == 0;
toRemoveEmpty(1:startline-1) = false;
newGcode = newGcode(~toRemoveEmpty);
%% output to file

fid = fopen([filename0,'_corners',fileext], 'wt');
fprintf(fid, '%s\n', newGcode);
fclose(fid);