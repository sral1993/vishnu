<?php

#author : Srivishnu Alvakonda
#name   : hw8.php
#purpose: A script to combine php and mysql
#date   : 2017/18/3
#version: 0.2 >

$header=htmlspecialchars("/var/www/html/hw8/header.php",ENT_QUOTES);
include_once($header);

$bcg = htmlspecialchars('background-color:#ffcc80',ENT_QUOTES);
echo "<body style=$bcg>";
echo "<h4 align='center'> <a style='color:#ff0000' href=index.php> Stories </a> &nbsp; &nbsp; &nbsp; &nbsp;
<a style='color:#ff0000' href=index.php?allc=1 > All Characters list</a> &nbsp; &nbsp; &nbsp; &nbsp;
<a style='color:#ff0000' href=add.php?addc=1 > Add Characters </a> </h4>";

echo "<hr>";

$include=htmlspecialchars("/var/www/html/hw8/hw8-lib.php",ENT_QUOTES);
include_once($include);

connect($db);

isset( $_REQUEST['s'])? $s = strip_tags($_REQUEST['s']) : $s=0;
isset( $_REQUEST['sid']) ? $sid = strip_tags($_REQUEST['sid']) : $sid=0;
isset( $_REQUEST['bid']) ? $bid = strip_tags($_REQUEST['bid']) : $bid=0;
isset( $_REQUEST['cid']) ? $cid = strip_tags($_REQUEST['cid']) : $cid=0;
isset( $_REQUEST['allc']) ? $allc = strip_tags($_REQUEST['allc']) : $allc=0;

if(!(is_numeric($s)))
{
$str= "Error!!! s is not numeric";
echo htmlspecialchars($str,ENT_QUOTES);
echo "<br>";
}
if(!(is_numeric($sid)))
{
$str= "Error!!! sid is not numeric";
echo htmlspecialchars($str,ENT_QUOTES);
echo "<br>";
}
if(!(is_numeric($bid)))
{
$str= "Error!!! bid is not numeric";
echo htmlspecialchars($str,ENT_QUOTES);
echo "<br>";
}
if(!(is_numeric($cid)))
{
$str= "Error!!! cid is not numeric";
echo htmlspecialchars($str,ENT_QUOTES);
echo "<br>";
}
if(!(is_numeric($allc)))
{
$str= "Error!!! allc is not numeric";
echo htmlspecialchars($str,ENT_QUOTES);
echo "<br>";
}

if(is_numeric($allc) && $allc != 1 && is_numeric($s) && is_numeric($sid) && is_numeric($bid) && is_numeric($cid))
{
switch($s) {
case 0;
default:
echo "<h4 style='color:#00cc99' align='center' > STORIES </h4>";
$query="SELECT storyid, story from stories";
$result=mysqli_query($db, $query);
echo "<br>";
echo "<table border='4' align = 'center'>";
echo "<tr>
<th> sid </th>
<th> Story </th>
</tr>";
while($row=mysqli_fetch_row($result)){
$row[0]=htmlspecialchars($row[0]);
$row[1]=htmlspecialchars($row[1]);
        echo "<tr>";
        echo "<td> $row[0] </td>";
        echo "<td> <a href=index.php?s=1&sid=$row[0] > $row[1] </a> </td>";
        echo "</tr>";
}
echo "</table>";
break;

case 1;
echo "<br>";
$que="SELECT story from stories where storyid=$sid";
$res=mysqli_query($db, $que);
$row1=mysqli_fetch_row($res);
echo "<h4 style='color:#00cc99' align='center' > Books in the Story : $row1[0]</h4>";
echo "<table border='4' align='center'>";
echo "<tr>
<th> bid  </th>
<th> Book </th>
</tr>";
$sid=mysqli_real_escape_string($db, $sid);
if ($stmt = mysqli_prepare($db, "SELECT bookid, title from books where storyid = ?")) {
	mysqli_stmt_bind_param($stmt, "s", $sid);
	mysqli_stmt_execute($stmt);
	mysqli_stmt_bind_result($stmt, $bid, $title);
	while(mysqli_stmt_fetch($stmt)) {
	$bid=htmlspecialchars($bid);
	$title=htmlspecialchars($title);
	echo "<tr>";
        echo "<td> $bid </td>";
        echo "<td> <a href=index.php?s=2&bid=$bid> $title </a> </td>";
        echo "</tr>";
}
mysqli_stmt_close($stmt);
}
echo "</table>";
break;

case 2;
echo "<br>";
$que="SELECT title from books where bookid=$bid";
$res=mysqli_query($db, $que);
$row1=mysqli_fetch_row($res);
echo "<h4 style='color:#00cc99' align='center' > Characters in the Book : $row1[0]</h4>";
echo "<table border='4' align = 'center'>";
echo "<tr>
<th> cid  </th>
<th> Character </th>
</tr>";
$bid=mysqli_real_escape_string($db, $bid);
if ($stmt = mysqli_prepare($db, "SELECT characters.characterid, name,Url from characters,appears,pictures where appears.bookid=? and characters.characterid=appears.characterid and characters.characterid = pictures.characterid")) {
        mysqli_stmt_bind_param($stmt, "s", $bid);
        mysqli_stmt_execute($stmt);
        mysqli_stmt_bind_result($stmt, $cid, $name, $Url);
        while(mysqli_stmt_fetch($stmt)) {
        $cid=htmlspecialchars($cid);
        $name=htmlspecialchars($name);
        echo "<tr>";
        echo "<td> $cid </td>";
        echo "<td> <a href=index.php?s=3&cid=$cid> $name </a> </td>";
        echo "</tr>";

}
mysqli_stmt_close($stmt);
}

echo "</table>";
break;

case 3;
echo "<br>";
$que="SELECT name from characters where characterid=$cid";
$res=mysqli_query($db, $que);
$row1=mysqli_fetch_row($res);
echo "<h4 style='color:#00cc99' align='center' > Books & Stories of the Character : $row1[0]</h4>";
echo "<table border='4' align = 'center'>";
echo "<tr>
<th> Book Title</th>
<th> Story </th>
</tr>";
$cid=mysqli_real_escape_string($db, $cid);
if ($stmt = mysqli_prepare($db, "SELECT title, story from books,appears,stories,characters where appears.characterid = ? and characters.characterid = appears.characterid and appears.bookid = books.bookid and books.storyid = stories.storyid")) {
        mysqli_stmt_bind_param($stmt, "s", $cid);
        mysqli_stmt_execute($stmt);
        mysqli_stmt_bind_result($stmt, $title, $name);
        while(mysqli_stmt_fetch($stmt)) {
        $title=htmlspecialchars($title);
        $name=htmlspecialchars($name);
        echo "<tr>";
        echo "<td> <a href=index.php> $title </a> </td>";
        echo "<td> <a href=index.php> $name </a> </td>";
        echo "</tr>";

}
mysqli_stmt_close($stmt);
}

echo "</table>";

break;
}
}

if($allc==1)
{
echo "<br>";
echo "<table border='4' align = 'center'>";
echo "<tr>
<th> Character </th>
<th> Pictures  </th>
</tr>";
$query="SELECT characters.characterid, name,Url from characters,pictures where characters.characterid=pictures.characterid";
$result=mysqli_query($db, $query);
while($row=mysqli_fetch_row($result)){
$row[0]=htmlspecialchars($row[0]);
$row[1]=htmlspecialchars($row[1]);
        echo "<tr>";
	echo "<td> <a href=index.php?s=3&cid=$row[0]> $row[1] </a> </td>";
        echo "<td> <img src=$row[2]> </td>";
        echo "</tr>";
}
echo "</table>";
}


?>

