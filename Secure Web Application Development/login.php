
<?php
$header=htmlspecialchars("/var/www/html/hw8/header.php",ENT_QUOTES);
include_once($header);

$bcg = htmlspecialchars('background-color:#ffcc80',ENT_QUOTES);
echo "<body style=$bcg>";
echo "<h4 align='center'> <a style='color:#ff0000'href=index.php>Stories </a> &nbsp; &nbsp; &nbsp; &nbsp;
<a style='color:#ff0000' href=index.php?allc=1 > All Characters list</a> &nbsp; &nbsp; &nbsp; &nbsp;
<a style='color:#ff0000' href=add.php?addc=1 > Add Characters </a> </h4>";

echo "<hr>";

$include=htmlspecialchars("/var/www/html/hw8/hw8-lib.php",ENT_QUOTES);
include_once($include);
connect($db);


echo "
<h4 align='center'>
<form method=post action=add.php?addc=1>
Username:
<input type= \"text\" name=\"postUser\">
<br>
Password:
<input type= \"password\" name=\"postPass\">
<br>
<input type= \"submit\" value=\"Login\" />
</form>
</h4>";

?>
