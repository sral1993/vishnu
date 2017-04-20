<?php
session_start();
session_regenerate_id();
$header=htmlspecialchars("/var/www/html/hw8/header.php",ENT_QUOTES);
include_once($header);

$bcg = htmlspecialchars('background-color:#ffcc80',ENT_QUOTES);
echo "<body style=$bcg>";
echo "<h4 align='center'> <a style='color:#ff0000'href=index.php> Stories </a> &nbsp; &nbsp; &nbsp; &nbsp;
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
isset( $_REQUEST['addc']) ? $addc = strip_tags($_REQUEST['addc']) : $addc=0;
isset( $_REQUEST['Character']) ? $Character = strip_tags($_REQUEST['Character']) : $Character="";
isset( $_REQUEST['Race']) ? $Race = strip_tags($_REQUEST['Race']) : $Race="";
isset( $_REQUEST['Side']) ? $Side = strip_tags($_REQUEST['Side']) : $Side="";
isset( $_REQUEST['picurl']) ? $picurl = strip_tags($_REQUEST['picurl']) : $picurl="";
isset( $_REQUEST['x']) ? $x = strip_tags($_REQUEST['x']) : $x="";
isset( $_REQUEST['postUser']) ? $postUser = strip_tags($_REQUEST['postUser']) : $postUser="";
isset( $_REQUEST['postPass']) ? $postPass = strip_tags($_REQUEST['postPass']) : $postPass="";
isset( $_REQUEST['username']) ? $username = strip_tags($_REQUEST['username']) : $username="";
isset( $_REQUEST['password']) ? $password = strip_tags($_REQUEST['password']) : $password="";
isset( $_REQUEST['salt']) ? $salt = strip_tags($_REQUEST['salt']) : $salt="";
isset( $_REQUEST['email']) ? $email = strip_tags($_REQUEST['email']) : $email="";
isset( $_REQUEST['id']) ? $id = strip_tags($_REQUEST['id']) : $id="";
isset( $_REQUEST['newpassword']) ? $newpassword = strip_tags($_REQUEST['newpassword']) : $newpassword="";



if(!isset($_SESSION['authenticated']))  {
 	$count=0;
	$x='fail';
	$a=$_SERVER['REMOTE_ADDR'];
	$whitelist=array('198.18.5.186');
	if(!in_array($a,$whitelist))    {
		$query="SELECT count(loginid) FROM login WHERE ip = ? and action = ? and date > DATE_SUB(NOW(), INTERVAL 1 HOUR)";
		if($stmt = mysqli_prepare($db, $query)) {
        		mysqli_stmt_bind_param($stmt, "ss", $a ,$x);
        		mysqli_stmt_execute($stmt);
        		mysqli_stmt_bind_result($stmt,$count);
			while(mysqli_stmt_fetch($stmt)) {
               			$count=htmlspecialchars($count);
        			}
			mysqli_stmt_close($stmt);
			}
			if($count >= 100)
			{        
 			header("Location: /hw8/login.php");
			}
			else
			{
			authenticate($db,$postUser,$postPass);
			}
		}
		else
		{
		authenticate($db,$postUser,$postPass);
		}
} 

checkAuth();




if(!(is_numeric($addc)))
{
$str= "Error!!! addc is not numeric";
echo htmlspecialchars($str,ENT_QUOTES);
echo "<br>";
}



if($addc==1)
{
echo " <h4 style='color:#00cc99' align='center'> Add New Character </h4>";
echo"
<h4  align='center'>
<form method=post action=add.php?s=4>
Character Name:
<input type= \"text\" name=\"Character\"> <br>
Race: &nbsp &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;
<input type= \"text\" name=\"Race\"> <br>
Side: &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; Good
<input type= \"radio\" name=\"Side\" value=\"Good\">  &nbsp; Evil
<input type= \"radio\" name=\"Side\" value= \"Evil\"> <br>
<input type= \"submit\" value =\"Submit\"/>
</form>
</h4>";
}

if(is_numeric($addc) && $addc !=1)
{
switch($s) {

case 4;
echo "<h4 style='color:#00cc99' align='center'>";
$Character=mysqli_real_escape_string($db, $Character);
$Race=mysqli_real_escape_string($db, $Race);
$Side=mysqli_real_escape_string($db, $Side);
if ($stmt = mysqli_prepare($db, "INSERT INTO characters set characterid='', name=?, race=?,side=?")) {
        mysqli_stmt_bind_param($stmt, "sss", $Character, $Race, $Side);
        mysqli_stmt_execute($stmt);
        mysqli_stmt_close($stmt);
}
if ($stmt = mysqli_prepare($db, "SELECT characterid from characters where
 name=? and race=? and side=? order by characterid desc limit 1")) {
        mysqli_stmt_bind_param($stmt, "sss" , $Character, $Race, $Side);
        mysqli_stmt_execute($stmt);
        mysqli_stmt_bind_result($stmt, $cid);
        while(mysqli_stmt_fetch($stmt)) {
        $cid=$cid;
        }
        mysqli_stmt_close($stmt);
} else {
        echo " Error with the query";
}

echo " Add Picture to $Character";
echo "
<form method=post action=add.php?s=5&cid=$cid&Character=$Character>
Picture URL:
<input type=\"text\" name=\"picurl\">
<input type=\"submit\" value = \"Submit\" />
</form>";
echo "</h4>";

break;

case 5;
echo "<h4 style='color:#00cc99' align='center'>";
$picurl=mysqli_real_escape_string($db, $picurl);
if ($stmt = mysqli_prepare($db, "INSERT INTO pictures set pictureid='', url=?,characterid=?")) {
        mysqli_stmt_bind_param($stmt, "ss", $picurl, $cid);
        mysqli_stmt_execute($stmt);
        mysqli_stmt_close($stmt);
} else {
        echo " Error with the query";
}
echo " Added picture for $Character";
echo"
<form method=post action=add.php?s=6&cid=$cid>
<input type=\"submit\" value =\"Add Character to Books\" />
</form>";

echo"</h4>";
break;

case 6;
echo "<h4 style='color:#00cc99' align='center'>";
if($x==1)
{
$bid=mysqli_real_escape_string($db, $bid);
if ($stmt = mysqli_prepare($db, "INSERT INTO appears set appearsid='',bookid=?,characterid=?")) {
        mysqli_stmt_bind_param($stmt, "ss", $bid, $cid);
        mysqli_stmt_execute($stmt);
        mysqli_stmt_close($stmt);
}
}
if($x==1)
{
echo"Added to the book: $bid";
}
echo"
<form method=post action=add.php?s=6&cid=$cid&x=1>
Please Select the Books: <br> <br>
<select name=\"bid\">";
if($stmt = mysqli_prepare($db, "SELECT distinct(a.bookid), b.title FROM books b, appears a WHERE a.bookid NOT IN(SELECT bookid FROM appears WHERE characterid=?) AND b.bookid=a.bookid" )) {
        echo"$cid";
	mysqli_stmt_bind_param($stmt,"s",$cid);
        mysqli_stmt_execute($stmt);
        mysqli_stmt_bind_result($stmt, $bookid, $title);
        while(mysqli_stmt_fetch($stmt)) {
        $bid=$bookid;
        $title=$title;
        echo "<option value=$bid> $title </option>";
}
mysqli_stmt_close($stmt);
}
        echo "</select>
        <input type=\"submit\" value=\"Add to Book\"/>
        </form>";
if($x==1)
 echo "<a href=\"index.php?s=3&cid=$cid\">Done</a>";
echo "</h4>";
break;

case 88;

session_destroy();
header("Location: /hw8/login.php");
break;

case 8;
if($_SESSION['userid']==1)
{
echo "<h4 style='color:#00cc99' align='center' > USERS LIST </h4>";
$query="SELECT userid, username from users";
$result=mysqli_query($db, $query);
echo "<br>";
echo "<table border='4' align = 'center'>";
echo "<tr>
<th> S.no </th>
<th> Users </th>
</tr>";
while($row=mysqli_fetch_row($result)){
$row[0]=htmlspecialchars($row[0]);
$row[1]=htmlspecialchars($row[1]);
        echo "<tr>";
        echo "<td> $row[0] </td>";
        echo "<td> $row[1] </td>";
        echo "</tr>";
}
echo "</table>";
}
else
{
echo " You dont have permission to list users buddy!!! Sorry!";
}
break;

case 90;

if($_SESSION['userid']==1)
{
echo "
<h4 align='center'>
<form method=post action=add.php?s=9>
username:
<input type= \"text\" name=\"username\">
<br>
Password:
<input type= \"password\" name=\"password\">
<br>
email: &nbsp; &nbsp; &nbsp; &nbsp;
<input type= \"text\" name=\"email\">
<br>
<br>
<input type= \"submit\" value=\"Submit\" />
</form>
</h4>";
}

else
{
echo " You don't have permission to access this page buddy!!!  I'm Sorry!";
}

break;

case 9;
if($_SESSION['userid']==1)
{
$username=mysqli_real_escape_string($db, $username);
$password=mysqli_real_escape_string($db, $password);
$email=mysqli_real_escape_string($db, $email);
$salt=rand(0,25689778967);
$password=hash('sha256',$password.$salt);
if ($stmt = mysqli_prepare($db, "INSERT INTO users set userid='', username=?, password=?, salt=?, email=?")) {
        mysqli_stmt_bind_param($stmt, "ssss", $username, $password, $salt, $email);
        mysqli_stmt_execute($stmt);
        mysqli_stmt_close($stmt);
echo"Added New user";

} else {
        echo " Error with the query";
}
}
else
{
  echo " You don't have permission to enter this page buddy!!! I'm Sorry!!";
}
break;


case 10;
if($_SESSION['userid']==1)
{
echo "
<h4 align='center'>
<form method=post action=add.php?s=11>
Userid: &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; 
<input type= \"text\" name=\"id\">
<br>
New Password:
<input type= \"password\" name=\"newpassword\">
<br>
<br>
<input type= \"submit\" value=\"Submit\" />
</form>
</h4>";
}
else
{
echo " You don't have permission to access this page buddy!!!  I'm Sorry!";
}
break;

case 11;
if($_SESSION['userid']==1)
{
if(is_numeric($id))
{
$id=mysqli_real_escape_string($db, $id);
$newpassword=mysqli_real_escape_string($db, $newpassword);
$salt=rand(0,25689778967);
$newpassword=hash('sha256',$newpassword.$salt);
if ($stmt = mysqli_prepare($db, "UPDATE users set password=?, salt=? WHERE userid=?")) {
        mysqli_stmt_bind_param($stmt, "sss", $newpassword, $salt, $id);
        mysqli_stmt_execute($stmt);
        mysqli_stmt_close($stmt);
echo"Updated New password for the userid:$id";

} else {
        echo " Error with the query";
}


}
else
{
echo "$id is not a valid userid";
}
}
else
{
  echo " You don't have permission to enter this page buddy!!! I'm Sorry!!";
}

break;

case 12;
if($_SESSION['userid']==1)
{
echo "<h4 style='color:#00cc99' align='center' > Failed Attempts </h4>";
$query="SELECT ip, count(loginid) from login where action='fail'group by ip";
$result=mysqli_query($db, $query);
echo "<br>";
echo "<table border='4' align = 'center'>";
echo "<tr>
<th> Ip </th>
<th> Failed Attempts </th>
</tr>";
while($row=mysqli_fetch_row($result)){
$row[0]=htmlspecialchars($row[0]);
$row[1]=htmlspecialchars($row[1]);
        echo "<tr>";
        echo "<td> $row[0] </td>";
        echo "<td> $row[1] </td>";
        echo "</tr>";
}
echo "</table>";
}
else
{
echo " You dont have permission to list users buddy!!! Sorry!";
}


break;
}
}

echo "<br>";
echo "<br>";
echo "<br>";
echo "<br>";
echo "
<h4 align='center'>
<a href=add.php?s=88> Logout </a>
</h4>";


if($_SESSION['userid']==1)
{
echo"
<h4 align='center'>
<a href=add.php?s=90> Add more Users </a>
<br>
</h4>";

echo"
<h4 align='center'>
<a href=add.php?s=8> List All Users </a>
</form>
</h4>";

echo"
<h4 align='center'>
<a href=add.php?s=10> Update Passwords </a>
</form>
</h4>";

echo"
<h4 align='center'>
<a href=add.php?s=12> Failed Login Summary </a>
</form>
</h4>";

}

?>
