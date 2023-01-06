import React,{ useState,useEffect,useRef } from "react"
import globalLinker from "../cloud/globalLinker";

if(localStorage.getItem('devList')==null){
    localStorage.setItem('devList',JSON.stringify([]));
}



// devLinker.linkerSetAdd(data=>console.log("devLinker >> ",data));

function stringToArray(inputString){
    var inputStringLength=inputString.length;
    var arrayIndexCounter=0;
    var finalArray=[];
    while(inputStringLength--)
        finalArray[arrayIndexCounter]=inputString.charCodeAt(arrayIndexCounter++);
    return finalArray;
}
function arrayToString(inputArray){
    var inputArrayLength=inputArray.length;
    var stringIndexCounter=0;
    var finalString="";
    while(inputArrayLength--)
        finalString+=String.fromCharCode(inputArray[stringIndexCounter++]);
    return finalString;
}

const base64ToArray16=(base64Data)=>{
    let array16=[];
    try{
        // console.log(base64Data.slice(0,base64Data.length-(base64Data.length%4)))
        let array8=stringToArray(atob(base64Data.slice(0,base64Data.length-(base64Data.length%4))));
        let loopCounter=(array8.length/2)+1;
        let array8Counter=0;
        while(loopCounter--){
            array16.push((array8[array8Counter++]||0)|((array8[array8Counter++]||0)<<8));
        }
    }catch(err){
        console.log('error >> ',err)
        return [];
    }
    // console.log(array16)
    return array16;
}

const plotBase64Array=({base64Array,height=150,scale=1.0})=>{
    let sampleList=base64ToArray16(base64Array);
    const highValue=0.15*height;
    const lowValue=0.85*height;

    let logicState=(sampleList[0]==0);
    const graphPlot=[];
    let delta=0;
    sampleList.slice(1).forEach(sample=>{
        graphPlot.push({
            y:logicState?highValue:lowValue,
            x:delta
        });
        graphPlot.push({
            y:logicState?highValue:lowValue,
            x:(delta+=(sample*scale))
        });
        logicState^=1;
    })
    return graphPlot;
}


const Graph=({plot})=>{

    const htmlCanvas=useRef();
    const canvasHight=plot.canvasHight||150;
    const canvasWidth=plot.canvasWidth||65535;    
    useEffect(()=>{
        const ctx=htmlCanvas.current.getContext('2d');
        ctx.fillStyle='black';
        ctx.fillRect(0, 0,canvasWidth,canvasHight);


        let lastPoint=null;
        (plot.data||[]).forEach(point=>{ 
            if(lastPoint!=null){
                ctx.beginPath();
                ctx.lineWidth = plot.width||2;
                ctx.strokeStyle = plot.color||'red';                
                ctx.moveTo(lastPoint.x, lastPoint.y);
                ctx.lineTo(point.x, point.y);
                ctx.stroke();
            }
            lastPoint=point;
        });
    },[plot])

    return(
        <>
            <div className='m-1 text overflow-scroll center border border-sky-800'>
                <canvas ref={htmlCanvas} height={canvasHight} width={canvasWidth}></canvas>
            </div>
        </>
    );
}




const AddDevice=({userDevice})=>{
    const deviceName=useRef();
    useEffect(()=>{
        deviceName.current.value="";
    },[userDevice.deviceName])
    return(
        <>
            <div className='m-1 p-1 text-left border border-sky-800 rounded max-w-max'>
                Add New Device <br />
                <input className="text-black" type="text" placeholder={`${userDevice.placeholder||'Device Name'}`} ref={deviceName}/>
                <button onClick={()=>{
                    userDevice.deviceName=deviceName.current.value;
                    userDevice.addDevice(deviceName.current.value);
                }} className='m-1 p-1 bg-sky-800 border hover:bg-sky-500 border-sky-800 rounded'>ADD</button>
            </div>
        </>
    );
}

const DeviceList=({deviceList})=>{
    const deviceRenderList=[];
    
    
    // const scaleFactorList=[];

    // let scaleFactorListCounter=(deviceList.userDevices||[]).length;
    // while(scaleFactorListCounter--)
    //     scaleFactorList.push(useRef());
    
    // const scaleFactor=useRef();
    (deviceList.userDevices||[]).forEach((device,index) => {
        if(device.deleted)
            return;
        device.index=index;

        // console.log(plotBase64Array(device.data));
        // console.log(" ----> ",stringToArray(atob(device.data)));


        deviceRenderList.push(
            <>
                <div className="m-1 p-1 overflow-scroll text-left border border-gray-300 bg-black rounded-md">
                    <span>{`${device.name||'Device Name'}`}</span><br />
                    <button onClick={()=>{
                        deviceList.delete(device);
                    }} className="m-1 p-1 bg-[#FF0000] border hover:bg-[#ff00006c] border-[#ff00006c] rounded inline-block float-left">Delete</button>
                    <button onClick={()=>{
                        deviceList.playBack(device);
                    }} className="m-1 p-1 bg-[#059862] border hover:bg-[#05986295] border-[#05986295] rounded inline-block float-right">Play Back</button>
                    <br />
                    <br />


                    <Graph plot={{
                        data:plotBase64Array({base64Array:device.data,scale:(device.graphScale||0.1)})
                    }}/>
                    {/* <input className="float-right m-1" type="range" min={0} max={100}  onChange={()=>{
                        console.log(this)
                    }}/> */}
                </div>
            </>
        );
    });
    return(
        <>
            {deviceRenderList}
        </>
    )
}

export default function SmartHub() {
    const [deviceName,clearNameInput]=useState();
    const [userDevices,setUserDevices]=useState(JSON.parse(localStorage.getItem('devList')));

    const hostServerConfig='https://raw.githubusercontent.com/engkhalil/xtensa32plus/main/dnsSquared.json';
    const globalUserCredentials='anNvbiBkaXJlY3RpdmVzIHRlc3Qg';

    const devLinker=new globalLinker(hostServerConfig,globalUserCredentials,()=>{
        // devLinker.linkerSend("connected\r\n")
    });
    
    let ADD_REQUEST="";


    devLinker.linkerSetAdd(deviceResponse=>{
        if((ADD_REQUEST!=="")&&(deviceResponse!=='CONNECTION-ALIVE-ACK')){
            let userDevList=JSON.parse(localStorage.getItem('devList'));        //^ array is expected
            userDevList.push({
                name:ADD_REQUEST,
                deleted:false,
                data:deviceResponse
            })
            localStorage.setItem('devList',JSON.stringify(userDevList));
            setUserDevices(JSON.parse(localStorage.getItem('devList')));
            ADD_REQUEST="";
        }
    })


    return (
        <>
            <AddDevice userDevice={{
                deviceName,
                addDevice:(userDeviceName)=>{
                    console.log(userDeviceName);
                    clearNameInput(userDeviceName);
                    ADD_REQUEST=userDeviceName;
                    devLinker.linkerSend('!get-key')
                }
            }}/>
            <DeviceList deviceList={{
                userDevices,
                delete:device=>{
                    // console.log(device);

                    let userDevList=JSON.parse(localStorage.getItem('devList'));        //^ array is expected
                    userDevList[device.index].deleted=true;
                    localStorage.setItem('devList',JSON.stringify(userDevList));
                    setUserDevices(JSON.parse(localStorage.getItem('devList')));

                },
                playBack:device=>{
                    // console.log(device);
                    devLinker.linkerSend(JSON.parse(localStorage.getItem('devList'))[device.index].data);
                }
            }}/>
        </>
    )
}
