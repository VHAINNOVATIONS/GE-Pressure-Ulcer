@ECHO OFF
:: Check WMIC is available
WMIC.EXE Alias /? >NUL 2>&1 || GOTO s_error

:: Use WMIC to retrieve date and time
FOR /F "skip=1 tokens=1-6" %%G IN ('WMIC Path Win32_LocalTime Get Day^,Hour^,Minute^,Month^,Second^,Year /Format:table') DO (
   IF "%%~L"=="" goto s_done
      Set _yyyy=%%L
      Set _mm=00%%J
      Set _dd=00%%G
      Set _hour=00%%H
      SET _minute=00%%I
      SET _second=00%%K
)
:s_done

:: Pad digits with leading zeros
      Set _mm=%_mm:~-2%
      Set _dd=%_dd:~-2%
      Set _hour=%_hour:~-2%
      Set _minute=%_minute:~-2%
      Set _second=%_second:~-2%

Set logtimestamp=%_yyyy%-%_mm%-%_dd%_%_hour%_%_minute%_%_second%
goto make_dump

:s_error
echo WMIC is not available, using default log filename
Set logtimestamp=_

:make_dump
echo "Please enter MySql root password"
set /p mysqlpass=
echo "Please enter copy destination directory"
set /p destdir=
if not exist %destdir% (
	echo "Destination directory not found. Exiting..."
	exit /b
)
"%MYSQL_HOME%\bin\mysql.exe" --user=root --password=%mysqlpass% --skip-column-names --batch --execute "select parameter_value from va_pupc.system_configuration where parameter_name = 'BASE_FILE_DIRECTORY';" > C:\tmp\base.txt
set /p basedir= < C:\tmp\base.txt
set FILENAME=assessment_db_dump_%logtimestamp%.sql
"%MYSQL_HOME%\bin\mysqldump.exe" --user=root --password=%mysqlpass% --result-file=%basedir%\%FILENAME% --skip-extended-insert --complete-insert --verbose va_pupc patient_identification braden_scores clinical_wound_assessment nutritional_status patient_admission patient_assessment system_assessment_experiment_measure system_assessment_experiment_recon system_assessment_experiment_segment system_assessment_experiment_temperature system_assessment_session treatment_plan wound_assessment_map
robocopy %basedir% %destdir%\backup_%logtimestamp% /e /xd resource /ns /nc /nfl /ndl
