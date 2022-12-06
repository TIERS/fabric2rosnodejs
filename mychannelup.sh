docker exec cli peer channel create -o orderer.example.com:7050 -c mychannel -f ./channel-artifacts/channel.tx --tls --cafile /opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/ordererOrganizations/example.com/orderers/orderer.example.com/msp/tlscacerts/tlsca.example.com-cert.pem
sleep 5

docker exec cli peer channel join -b mychannel.block

docker exec -e CORE_PEER_ADDRESS=peer1.org1.example.com:8051 -e CORE_PEER_TLS_ROOTCERT_FILE=/opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/peerOrganizations/org1.example.com/peers/peer1.org1.example.com/tls/ca.crt cli peer channel join -b mychannel.block

docker exec -e CORE_PEER_MSPCONFIGPATH=/opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/peerOrganizations/org2.example.com/users/Admin@org2.example.com/msp -e CORE_PEER_ADDRESS=peer0.org2.example.com:9051 -e CORE_PEER_LOCALMSPID="Org2MSP" -e CORE_PEER_TLS_ROOTCERT_FILE=/opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/peerOrganizations/org2.example.com/peers/peer0.org2.example.com/tls/ca.crt cli peer channel join -b mychannel.block

docker exec cli peer channel update -o orderer.example.com:7050 -c mychannel -f ./channel-artifacts/Org1MSPanchors.tx --tls --cafile /opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/ordererOrganizations/example.com/orderers/orderer.example.com/msp/tlscacerts/tlsca.example.com-cert.pem

docker exec -e CORE_PEER_MSPCONFIGPATH=/opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/peerOrganizations/org2.example.com/users/Admin@org2.example.com/msp -e CORE_PEER_ADDRESS=peer0.org2.example.com:9051 -e CORE_PEER_LOCALMSPID="Org2MSP" -e CORE_PEER_TLS_ROOTCERT_FILE=/opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/peerOrganizations/org2.example.com/peers/peer0.org2.example.com/tls/ca.crt cli peer channel update -o orderer.example.com:7050 -c mychannel -f ./channel-artifacts/Org2MSPanchors.tx --tls --cafile /opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/ordererOrganizations/example.com/orderers/orderer.example.com/msp/tlscacerts/tlsca.example.com-cert.pem

# Package chaincode
pushd ../chaincode/simplecc
GO111MODULE=on go mod vendor
popd

docker exec cli peer lifecycle chaincode package simplecc.tar.gz --path /opt/gopath/src/github.com/chaincode/simplecc --label simplecc_1

# Install chaincode package to all peers
# peer0.org1
docker exec cli peer lifecycle chaincode install simplecc.tar.gz
# peer1.org1
docker exec -e CORE_PEER_ADDRESS=peer1.org1.example.com:8051 -e CORE_PEER_TLS_ROOTCERT_FILE=/opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/peerOrganizations/org1.example.com/peers/peer1.org1.example.com/tls/ca.crt cli peer lifecycle chaincode install simplecc.tar.gz
# peer0.org2
docker exec -e CORE_PEER_MSPCONFIGPATH=/opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/peerOrganizations/org2.example.com/users/Admin@org2.example.com/msp -e CORE_PEER_ADDRESS=peer0.org2.example.com:9051 -e CORE_PEER_LOCALMSPID="Org2MSP" -e CORE_PEER_TLS_ROOTCERT_FILE=/opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/peerOrganizations/org2.example.com/peers/peer0.org2.example.com/tls/ca.crt cli peer lifecycle chaincode install simplecc.tar.gz

# Approve chaincode for both organizations
# org1
docker exec cli peer lifecycle chaincode approveformyorg --tls --cafile /opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/ordererOrganizations/example.com/orderers/orderer.example.com/msp/tlscacerts/tlsca.example.com-cert.pem --channelID mychannel --name simplecc --version 1 --sequence 1 --waitForEvent --package-id simplecc_1:dd2793f9ce4d2828e1b9fceff66fb0319f5e05700f477c5b0c51e3fd9a0c6d0e
# org2
docker exec -e CORE_PEER_MSPCONFIGPATH=/opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/peerOrganizations/org2.example.com/users/Admin@org2.example.com/msp -e CORE_PEER_ADDRESS=peer0.org2.example.com:9051 -e CORE_PEER_LOCALMSPID="Org2MSP" -e CORE_PEER_TLS_ROOTCERT_FILE=/opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/peerOrganizations/org2.example.com/peers/peer0.org2.example.com/tls/ca.crt cli peer lifecycle chaincode approveformyorg --tls --cafile /opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/ordererOrganizations/example.com/orderers/orderer.example.com/msp/tlscacerts/tlsca.example.com-cert.pem --channelID mychannel --name simplecc --version 1 --sequence 1 --waitForEvent --package-id simplecc_1:dd2793f9ce4d2828e1b9fceff66fb0319f5e05700f477c5b0c51e3fd9a0c6d0e

# Check approval status
docker exec cli peer lifecycle chaincode checkcommitreadiness --channelID mychannel --name simplecc --version 1 --sequence 1 

# Commit chaincode
docker exec cli peer lifecycle chaincode commit -o orderer.example.com:7050 --tls --cafile /opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/ordererOrganizations/example.com/orderers/orderer.example.com/msp/tlscacerts/tlsca.example.com-cert.pem --peerAddresses peer0.org1.example.com:7051 --tlsRootCertFiles /opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/peerOrganizations/org1.example.com/peers/peer0.org1.example.com/tls/ca.crt --peerAddresses peer0.org2.example.com:9051 --tlsRootCertFiles /opt/gopath/src/github.com/hyperledger/fabric/peer/crypto/peerOrganizations/org2.example.com/peers/peer0.org2.example.com/tls/ca.crt --channelID mychannel --name simplecc --version 1 --sequence 1 

# Check commit status
docker exec cli peer lifecycle chaincode querycommitted --channelID mychannel --name simplecc
