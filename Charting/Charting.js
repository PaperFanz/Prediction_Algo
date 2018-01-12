var board_canvas, board_context;  
var interval;
var HEIGHT=500, WIDTH=1000;
var COLOR = new Array("#e00", "#0e0", "#00e", "#0ee", "#e0e", "#ee0");
var lastPM = new Array();
var newPM = new Array();
var avgPM = new Array();

function start_PM() {
  board_canvas = document.getElementById("Board");
  board_context = board_canvas.getContext("2d");
  board_context.clearRect(0,0,WIDTH,HEIGHT);
  board_context.beginPath();
  for (var i=1; i<10; i++ ) {
    board_context.moveTo( 0, i*HEIGHT/10 );
    board_context.lineTo( WIDTH, i*HEIGHT/10 );
    board_context.moveTo( i*WIDTH/10, 0 );
    board_context.lineTo( i*WIDTH/10, HEIGHT );
  }
  board_context.strokeStyle = "#ccc";
  board_context.stroke();
      board_context.fillStyle = "#000";
  for ( var i=1; i<10; i++ )
    board_context.fillText( i, i*WIDTH/10-8, HEIGHT-2 );
  for ( var i=0; i<10; i++ )
    board_context.fillText( i, 2, HEIGHT-i*HEIGHT/10-2 );

  var mode = document.getElementById('mode').value;
  var scale = parseFloat(document.getElementById('scale').value);
  var offset = parseInt(document.getElementById('offset').value);
  var data0 = document.getElementById('data').value.split('\12');
  var data1 = new Array();
  var data_min = new Array();
  for ( var j=0; j<9; j++ ) data_min[j] = 1000;
  for ( var i=0; i*10<data0.length; i++ ) {
    newPM = data0[i*10+1].split('\t');
    data1[i]=new Array();
    for ( var j=0; j<newPM.length; j++ ) {
        data1[i][j]= parseInt(newPM[j]);
        if ( data1[i][j]<data_min[j] )
            data_min[j] = data1[i][j];
    }
  }
  for ( var i=0; i<data1.length; i++ )
    for ( var j=0; j<3; j++ ) {
      data1[i][j] -= data_min[j];
    }

  for ( var i=offset; i<data1.length; i++ ) {
    if ( mode=="Shape") {
      board_context.beginPath();
      board_context.strokeStyle = COLOR[0];
      if ( i==0 ) 
        board_context.moveTo(data1[0][0]*scale, data1[0][1]*scale);
      else
        board_context.moveTo(data1[i-1][0]*scale, HEIGHT-data1[i-1][1]*scale/2);
      board_context.lineTo(data1[i][0]*scale, HEIGHT-data1[i][1]*scale/2);
      board_context.stroke();
    }
    var x = i-offset;
    if ( mode=="Position") for ( var j=0; j<3; j++ ) {
      board_context.beginPath();
      board_context.strokeStyle = COLOR[j];
      if ( x==0 ) 
        board_context.moveTo(x, HEIGHT);
      else
        board_context.moveTo(x, HEIGHT-data1[i-1][j]/2*scale);
      board_context.lineTo(x+1, HEIGHT-data1[i][j]/2*scale);
      board_context.stroke();
    }
    if ( mode=="Velocity") for ( var j=3; j<6; j++ ) {
      board_context.beginPath();
      board_context.strokeStyle = COLOR[j-3];
      if ( x==0 ) 
        board_context.moveTo(x, HEIGHT);
      else
        board_context.moveTo(x, HEIGHT-data1[i-1][j]/2*scale);
      board_context.lineTo(x+1, HEIGHT-data1[i][j]/2*scale);
      board_context.stroke();
    }
    if ( mode=="Acceleration") for ( var j=3; j<6; j++ ) {
      board_context.beginPath();
      board_context.strokeStyle = COLOR[j-3];
      if ( x==0 || x==1 ) 
        board_context.moveTo(x, HEIGHT);
      else
        board_context.moveTo(x, HEIGHT-(data1[i-1][j]-data1[i-2][j])/2*scale);
      if ( i>1 ) board_context.lineTo(x+1, HEIGHT-(data1[i][j]-data1[i-1][j])/2*scale);
      board_context.stroke();
    }
  }
}
