function results = run_stage4()
%RUN_STAGE4  One command for Stage 4 (Pass 1):
%   1. s4_01_build_codegen_model : builds standalone discrete controller/ctrl_codegen.slx
%   2. s4_02_generate_code       : generates C code, copies snapshot to 04_firmware/app_generated/,
%                                  writes 04_firmware/app_wrapper.{c,h} & prof.h, audits for double/malloc
%   3. s4_03_run_sil_b2b         : compiles generated C into native MEX SIL and runs 5 MIL-vs-SIL
%                                  back-to-back scenarios (>7,500 ticks total), writes b2b_report.md
setup_paths();
s4_01_build_codegen_model();
cg = s4_02_generate_code();
sil = s4_03_run_sil_b2b();

maxFlt = max([sil.scenarios.fltDiff]);
maxInt = max([sil.scenarios.intDiff]);

chk = {
 'A4-1 SIL vs MIL float signals within 1e-4',     sil.allPass && maxFlt <= 1e-4, sprintf('max |diff| = %.2e', maxFlt)
 'A4-1b SIL vs MIL int/enum signals exact (0)',   maxInt == 0,                   sprintf('max |diff| = %d', maxInt)
 'A4-6 No double (real_T) in generated step()',   ~cg.hasDoubleInStep,           sprintf('doubleInStep = %d', cg.hasDoubleInStep)
 'A4-6b No dynamic memory (malloc/free) in C',    ~cg.hasDynMem,                 sprintf('dynMem = %d', cg.hasDynMem)
 'A4-7 Clean snapshot in 04_firmware/app_generated', isfile(cg.cFile),           sprintf('%d C/H files exported', cg.numFiles)
};

fprintf('\n%-48s %-6s %s\n', 'Stage 4 Criterion', 'Result', 'Value');
ok = true;
for i = 1:size(chk,1)
    fprintf('%-48s %-6s %s\n', chk{i,1}, tern(chk{i,2},'PASS','FAIL'), chk{i,3});
    ok = ok && chk{i,2};
end
fprintf('\nStage 4 (Codegen & SIL): %s\n', tern(ok,'ALL PASS','SOME FAIL'));
results = struct('codegen', cg, 'sil', sil, 'ok', ok);
end

function s = tern(c,a,b), if c, s=a; else, s=b; end, end
