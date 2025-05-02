% cleanupSimulinkArtifacts
%   この関数は、カレントフォルダ内の以下を削除します。
%     1. slprj フォルダとその配下
%     2. *.slxc ファイル
%     3. *.mex* ファイル
%   最後に Simulink.fileGenControl をリセットして設定を初期状態に戻します。

% 1. slprj フォルダを削除
if isfolder('slprj')
    % 's' オプションで再帰的に削除
    rmdir('slprj', 's');    % :contentReference[oaicite:0]{index=0}
    fprintf('Deleted folder: slprj\n');
end

% 2. .slxc ファイルをすべて削除
slxcFiles = dir('*.slxc');
if ~isempty(slxcFiles)
    delete('*.slxc');      % :contentReference[oaicite:1]{index=1}
    fprintf('Deleted %d .slxc files\n', numel(slxcFiles));
end

% 3. MEX ファイルをすべて削除 (*.mexw64, *.mexa64 など)
mexFiles = dir('*.mex*');
if ~isempty(mexFiles)
    delete('*.mex*');      % :contentReference[oaicite:2]{index=2}
    fprintf('Deleted %d MEX files\n', numel(mexFiles));
end

% 4. Simulink.fileGenControl の設定をリセット
%    これにより、キャッシュおよびコード生成フォルダ設定を元に戻す
Simulink.fileGenControl('reset');  % :contentReference[oaicite:3]{index=3}
fprintf('Simulink.fileGenControl has been reset\n');