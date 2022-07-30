<?php
// it is web side example for easy entry system  
// return only "y" or "n"
// place this file into your we server etc. into /var/www/html
if ( $_REQUEST["q"]=="check" ) {
    $rader=$_REQUEST["reader"];
    $card_code=$_REQUEST["card_code"];
    if ($card_code=="0260C3E5B9") echo "y";
    else echo "n";
}
php?>